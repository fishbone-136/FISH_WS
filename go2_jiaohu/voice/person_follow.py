"""Low-camera person following geometry; no camera, DDS or motor ownership."""
from dataclasses import dataclass
import math
import threading

import cv2
import numpy as np


@dataclass
class DepthCloud:
    body: np.ndarray
    height: np.ndarray
    depth: np.ndarray
    u: np.ndarray
    v: np.ndarray
    valid: np.ndarray


@dataclass
class PersonMeasurement:
    point: tuple
    pixels: np.ndarray
    distance: float
    image_bearing: float | None = None  # signed error from the horizontal image centre


def camera_projection(rs, intrinsics, extrinsics=None):
    """Project either RGB or wide IR into the calibrated COLOR optical frame.

    Cache unit rays using the SDK's actual distortion model. Translation must
    be applied after depth scaling, not baked into the unit rays.
    """
    rotation = np.eye(3) if extrinsics is None else np.array(extrinsics.rotation).reshape(3,3,order='F')
    translation = np.zeros(3) if extrinsics is None else np.array(extrinsics.translation)
    rays = np.full((intrinsics.height, intrinsics.width, 3), np.nan, np.float32)
    lock = threading.Lock()

    def one(u, v, z):
        point = rs.rs2_deproject_pixel_to_point(intrinsics, [u,v], z)
        return tuple(np.asarray(point) @ rotation.T + translation)

    def many(u, v, z):
        u, v, z = np.asarray(u,dtype=int), np.asarray(v,dtype=int), np.asarray(z)
        with lock:
            missing = ~np.isfinite(rays[v,u,0])
            if missing.any():
                rays[v[missing],u[missing]] = [rs.rs2_deproject_pixel_to_point(
                    intrinsics, [float(x),float(y)], 1.) for x,y in zip(u[missing],v[missing])]
            unit = rays[v,u].copy()
        return (unit*z[:,None]) @ rotation.T + translation
    return one, many


def person_appearance(frame, box):
    """Include intensity: H/S histograms alone are identical for all IR images."""
    h,w = frame.shape[:2]
    x1,y1,x2,y2 = box
    roi = frame[int((y1+.15*(y2-y1))*h):int((y2-.15*(y2-y1))*h),
                int((x1+.15*(x2-x1))*w):int((x2-.15*(x2-x1))*w)]
    if roi.size == 0:
        raise ValueError('人物区域太小')
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray],[0],None,[32],[0,256])
    return cv2.normalize(hist,hist).flatten()


def match_person(boxes, previous, frame, original):
    from voice.near_approach import box_iou
    cx,cy = (previous[0]+previous[2])/2, (previous[1]+previous[3])/2
    width,height = previous[2]-previous[0], previous[3]-previous[1]
    candidates = []
    for candidate in boxes:
        box = tuple(candidate)
        bx,by = (box[0]+box[2])/2, (box[1]+box[3])/2
        area_ratio = (box[2]-box[0])*(box[3]-box[1])/(width*height)
        if (box_iou(box,previous) >= .15 and .4 <= area_ratio <= 2.5
                and abs(bx-cx) <= max(.10,width*.9) and abs(by-cy) <= max(.10,height*.5)
                and cv2.compareHist(person_appearance(frame,box),original,cv2.HISTCMP_CORREL) >= .45):
            candidates.append(box)
    if len(candidates) != 1:
        raise ValueError('人物匹配丢失或不唯一，已停止')
    box = candidates[0]
    if any(box_iou(box,other) > .12 for other in boxes if tuple(other) != box):
        raise ValueError('多人交叉遮挡，已停止')
    return box


class TargetUncertain(ValueError):
    """No motion may use this frame; keep the last confirmed target for recovery."""


class PersonTrack:
    """Associate detections over time using prediction, measured depth and appearance."""
    def __init__(self):
        self.box = self.appearance = self.anchor = self.distance = self.last_seen = None
        self.velocity = np.zeros(2)
        self._pending = None

    def update(self, boxes, snapshot, cloud, project):
        from voice.near_approach import box_iou
        from voice.person_detector import select_person
        now = snapshot.captured_monotonic
        if not boxes:
            self._pending = None
            raise TargetUncertain('人物漏检')
        if self.box is None:
            try:
                box = select_person(boxes,'any')
                result = measure_person(snapshot,box,cloud,project=project)
            except ValueError as error:
                raise TargetUncertain(str(error)) from error
            features = person_appearance(snapshot.color,box)
        else:
            elapsed = now-self.last_seen
            if not 0. < elapsed <= 1.2:
                raise TargetUncertain('目标确认间隔过长')
            old = np.array([(self.box[0]+self.box[2])/2,(self.box[1]+self.box[3])/2])
            predicted = old+np.clip(self.velocity*min(elapsed,.6),-.15,.15)
            width,height = self.box[2]-self.box[0],self.box[3]-self.box[1]
            gate_x = min(.28,max(.10,width*.8)+elapsed*.25)
            gate_y = min(.45,max(.20,height*.9)+elapsed*.15)
            jump = min(.8,.30+elapsed*.8)
            candidates = []
            for candidate in boxes:
                box = tuple(float(x) for x in candidate)
                if not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1):
                    continue
                centre = np.array([(box[0]+box[2])/2,(box[1]+box[3])/2])
                dx,dy = np.abs(centre-predicted)
                area_ratio = (box[2]-box[0])*(box[3]-box[1])/(width*height)
                if dx > gate_x or dy > gate_y or not .15 <= area_ratio <= 6.:
                    continue
                try:
                    result = measure_person(snapshot,box,cloud,self.distance,project=project,max_jump=jump)
                    features = person_appearance(snapshot.color,box)
                except ValueError:
                    continue
                similarity = max(cv2.compareHist(features,self.appearance,cv2.HISTCMP_CORREL),
                                 cv2.compareHist(features,self.anchor,cv2.HISTCMP_CORREL))
                overlap = box_iou(box,self.box)
                depth_error = abs(result.distance-self.distance)
                # An abrupt appearance change is accepted only with very strong
                # short-term geometry, not as a long-occlusion identity guess.
                if similarity < .25 and not (elapsed <= .25 and overlap >= .4 and depth_error <= .2):
                    continue
                geometry = 1.-.5*(dx/gate_x+dy/gate_y)
                score = (.4*geometry+.25*(1.-depth_error/jump)+.20*overlap
                         +.15*max(0.,similarity))
                if score >= .55:
                    candidates.append((score,box,result,features))
            candidates.sort(key=lambda item:item[0],reverse=True)
            if not candidates:
                self._pending = None
                raise TargetUncertain('人物暂时失配（位置/深度/外观），已停车等待')
            if len(candidates) > 1 and candidates[0][0]-candidates[1][0] < .12:
                self._pending = None
                raise TargetUncertain('多人候选相近，已停车等待目标明确')
            _,box,result,features = candidates[0]
            if elapsed > .25:
                confirmed = (self._pending is not None
                             and 0 < now-self._pending[0] <= .3
                             and box_iou(box,self._pending[1]) >= .3
                             and abs(result.distance-self._pending[2]) <= .20)
                self._pending = (now,box,result.distance)
                if not confirmed:
                    raise TargetUncertain('重新发现目标，等待连续两帧确认')
            centre = np.array([(box[0]+box[2])/2,(box[1]+box[3])/2])
            scale_ratio = (box[2]-box[0])*(box[3]-box[1])/(width*height)
            if .65 <= scale_ratio <= 1.6:
                velocity = np.clip((centre-old)/elapsed,-.8,.8)
                self.velocity = .6*self.velocity+.4*velocity
            else:
                self.velocity *= .5  # Full/partial box changes are not body motion.
        if self.appearance is None:
            self.anchor = features.copy()
            self.appearance = features.copy()
        else:
            self.appearance = cv2.normalize(.9*self.appearance+.1*features,None).flatten()
        self.box,self.distance,self.last_seen = box,result.distance,now
        self._pending = None
        return box,result


def depth_cloud(snapshot, state, project, stride=6):
    """Keep image topology while projecting a subsampled aligned depth image."""
    depth = np.asarray(snapshot.depth_mm)
    if depth.ndim != 2 or depth.shape != snapshot.color.shape[:2]:
        raise ValueError('需要同步对齐的 RGB-D 画面')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[v, u].astype(float)*.001
    valid = np.isfinite(z) & (z >= .25) & (z <= 6.)
    optical = np.zeros((*z.shape, 3))
    if valid.any():
        if snapshot.deproject_many is not None:
            optical[valid] = snapshot.deproject_many(u[valid],v[valid],z[valid])
        else:
            optical[valid] = [snapshot.deproject(float(x), float(y), float(d))
                              for x, y, d in zip(u[valid], v[valid], z[valid])]
    body = project(optical)
    valid &= np.isfinite(body).all(axis=-1)
    roll, pitch = state['rpy'][:2]
    height = (-math.sin(pitch)*body[..., 0]
              + math.cos(pitch)*math.sin(roll)*body[..., 1]
              + math.cos(pitch)*math.cos(roll)*body[..., 2] + state['height'])
    return DepthCloud(body, height, z, u, v, valid)


def measure_person(snapshot, box, cloud, previous=None, project=None, max_jump=.30):
    """Find supported foreground components, rejecting floor and broad background.

    Components are grown over the whole image, not just the person box, so a
    wall seen through the leg gap is rejected when it extends outside the box.
    This is depth foreground extraction, not semantic body-part segmentation.
    """
    h, w = snapshot.depth_mm.shape
    x1, y1, x2, y2 = box
    if not all(math.isfinite(x) for x in box) or not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        raise ValueError('人物框无效')
    bw, bh = (x2-x1)*w, (y2-y1)*h
    roi = ((cloud.u >= x1*w+.05*bw) & (cloud.u <= x2*w-.05*bw)
           & (cloud.v >= y1*h+.05*bh) & (cloud.v <= y2*h-.05*bh))
    foreground = cloud.valid & (cloud.height > .10) & (cloud.height < 2.2)
    values = cloud.depth[foreground & roi]
    if values.size < 16:
        raise ValueError('人物区域有效深度不足')
    bins, edges = np.histogram(values, bins=np.arange(.2, 6.21, .1))
    min_support = max(8,np.count_nonzero(roi)*.015)
    padded = ((cloud.u >= x1*w-.10*bw) & (cloud.u <= x2*w+.10*bw)
              & (cloud.v >= y1*h-.10*bh) & (cloud.v <= y2*h+.10*bh))
    indices = np.flatnonzero(bins >= 8)
    if previous is not None:
        indices = sorted(indices,key=lambda i:abs((edges[i]+edges[i+1])*.5-previous))
    jumped = False
    out_of_range = False
    # Prefer a temporally consistent supported surface, never an isolated minimum.
    for index in indices:
        center = (edges[index]+edges[index+1])*.5
        mask = foreground & (np.abs(cloud.depth-center) <= .12)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        selected = np.zeros(mask.shape, bool)
        for label in range(1, count):
            if stats[label,cv2.CC_STAT_AREA] < min_support:
                continue
            component = labels == label
            inside = component & roi
            if np.count_nonzero(inside) < min_support:
                continue
            if np.ptp(cloud.v[inside]) < bh*.18:
                continue
            if np.count_nonzero(component & padded) < stats[label, cv2.CC_STAT_AREA]*.85:
                continue
            selected |= inside
        if np.count_nonzero(selected) < 16:
            continue
        point = tuple(np.median(cloud.body[selected], axis=0))
        image_bearing = None
        if project is not None:
            # Sample range on the legs, but aim at the box centre. A larger
            # number of points on one swinging leg must not steer the robot.
            z = float(np.median(cloud.depth[selected]))
            v = float(np.median(cloud.v[selected]))
            optical = snapshot.deproject((x1+x2)*w*.5,v,z)
            point = tuple(project(optical))
            centre = project(snapshot.deproject(w*.5,v,z))
            difference = math.atan2(point[1],point[0])-math.atan2(centre[1],centre[0])
            image_bearing = math.atan2(math.sin(difference),math.cos(difference))
        distance = math.hypot(*point[:2])
        if not .45 <= distance <= 4. or point[0] <= 0:
            out_of_range = True
            continue
        if previous is not None and abs(distance-previous) > max_jump:
            jumped = True
            continue
        pixels = np.column_stack((cloud.u[selected], cloud.v[selected]))
        return PersonMeasurement(point, pixels, distance, image_bearing)
    if jumped:
        raise ValueError('人物距离跳变，需重新锁定')
    if out_of_range:
        raise ValueError('人物超出 0.45–4 米测距范围')
    raise ValueError('无法区分腿部前景和背景，请调整站位')


class FollowFilter:
    """Smooth gait variation, keep a stop band, and limit acceleration."""
    def __init__(self, stand_off=1.5, max_speed=.35):
        if not math.isfinite(stand_off) or not 1. <= stand_off <= 2.:
            raise ValueError('跟随距离应在 1–2 米之间')
        if not math.isfinite(max_speed) or not .05 <= max_speed <= .40:
            raise ValueError('跟随速度应在 0.05–0.40 米/秒之间')
        self.stand_off, self.max_speed = stand_off, max_speed
        self.distance = self.last_time = None
        self.speed = 0.
        self.walking = False
        self.turning = False
        self.turn_integral = 0.
        self.last_bearing = None

    def reset(self):
        self.distance = self.last_time = None
        self.speed = 0.
        self.walking = False
        self.turning = False
        self.turn_integral = 0.
        self.last_bearing = None

    def velocity(self, measurement, now, yaw_speed=None):
        distance = measurement.distance
        body_bearing = math.atan2(measurement.point[1], measurement.point[0])
        bearing = measurement.image_bearing if measurement.image_bearing is not None else body_bearing
        if not all(math.isfinite(v) for v in (distance,bearing,body_bearing,now)):
            raise ValueError('人物距离或朝向反馈无效')
        if abs(body_bearing) > math.radians(45) or abs(bearing) > math.radians(45):
            raise ValueError('人物偏离前方超过 45°，请重新锁定')
        elapsed = .1 if self.last_time is None else now-self.last_time
        dt = min(.2,max(0.,elapsed))
        closing_rate = 0.
        if self.last_bearing is not None and .02 <= elapsed <= .3:
            closing_rate = max(0.,-math.copysign(1.,bearing)*(bearing-self.last_bearing)/elapsed)
        self.last_bearing = bearing
        if yaw_speed is not None and not math.isfinite(yaw_speed):
            raise ValueError('机器狗转向反馈无效')
        self.last_time = now
        alpha = 1.-math.exp(-dt/.20)
        self.distance = distance if self.distance is None else self.distance+alpha*(distance-self.distance)
        # Raw near measurements override smoothing: never continue on old range.
        if distance <= self.stand_off+.06 or self.distance <= self.stand_off+.06:
            self.walking = False
        elif self.distance >= self.stand_off+.12:
            self.walking = True
        desired = min(self.max_speed, .8*max(0., self.distance-self.stand_off-.04)) if self.walking else 0.
        # Keep the person centred before spending speed on closing the range.
        if abs(bearing) >= math.radians(12):
            desired = 0.
        else:
            desired *= math.cos(bearing)**2
            if abs(bearing) > math.radians(5):
                desired *= (math.radians(12)-abs(bearing))/math.radians(7)
        self.speed = min(desired, self.speed+.60*dt) if desired else 0.
        # Schmitt trigger prevents alternating corrections around the image centre.
        if abs(bearing) <= math.radians(.6):
            self.turning = False
            self.turn_integral = 0.
        elif abs(bearing) >= math.radians(1.2):
            self.turning = True
        if self.turn_integral*bearing < 0:
            self.turn_integral = 0.
        wz = 0.
        if self.turning:
            braking = (yaw_speed is not None and yaw_speed*bearing > 0 and abs(yaw_speed) > .04
                       and abs(bearing) <= math.radians(.6)+.20*min(.5,closing_rate))
            if abs(bearing) < math.radians(12) and not braking:
                self.turn_integral = max(-.25,min(.25,self.turn_integral+bearing*dt))
            raw = bearing*3.2+self.turn_integral
            # Small commands in the logs did not produce useful motion. Apply
            # bounded breakaway compensation, then stop fully inside the band.
            wz = math.copysign(min(.50,max(.22,abs(raw))),bearing)
            if braking:
                wz = 0.
        if distance < .65:
            wz = 0.
            self.turning = False
            self.turn_integral = 0.
        return self.speed, wz


def check_path(cloud, vx, wz):
    """Short swept front corridor, including steering, with observed floor."""
    central = (cloud.u >= cloud.u.max()*.25) & (cloud.u <= cloud.u.max()*.75)
    central &= (cloud.v >= cloud.v.max()*.4) & (cloud.v <= cloud.v.max()*.85)
    if not central.any() or np.mean(cloud.valid[central]) < .65:
        raise ValueError('前方深度空洞过多，已停止')
    x, y = cloud.body[..., 0], cloud.body[..., 1]
    floor = cloud.valid & (x > .2) & (x < 2.) & (np.abs(y) < .5) & (np.abs(cloud.height) < .065)
    if np.count_nonzero(floor) < 12:
        raise ValueError('无法确认前方地面，已停止')
    obstacles = cloud.valid & (cloud.height > .08) & (cloud.height < 1.6)
    for t in np.linspace(0., 1.2, 7):
        yaw = wz*t
        px = vx*t if abs(wz) < 1e-6 else vx/wz*math.sin(yaw)
        py = 0. if abs(wz) < 1e-6 else vx/wz*(1.-math.cos(yaw))
        dx, dy = x-px, y-py
        forward = math.cos(yaw)*dx+math.sin(yaw)*dy
        lateral = -math.sin(yaw)*dx+math.cos(yaw)*dy
        swept = obstacles & (forward > -.35) & (forward < .60) & (np.abs(lateral) < .42)
        if np.count_nonzero(swept) >= 3:
            raise ValueError('前方或转弯通道存在障碍，已停止')

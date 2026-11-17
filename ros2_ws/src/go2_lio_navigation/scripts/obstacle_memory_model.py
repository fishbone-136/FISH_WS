"""Retain blind landmarks; clear with repeated geometry and motion evidence."""
import math
from copy import deepcopy
import numpy as np
from scipy.spatial import cKDTree


def clear_confirmed_region(data,pose,bounds=(-.5,.5,-.4,-.2),minimum_age=5.,allowed_hits=None):
    """Explicit operator-confirmed free space; never automatic blind expiry.

    Preserve fresh hits, all geometry outside the confirmed body-frame box,
    and the original checkpoint for rollback. The caller must verify that the
    robot is stationary in the same LIO session and the area is physically free.
    """
    if len(pose)!=3 or not all(math.isfinite(v) for v in (*pose,*bounds,minimum_age)):
        raise ValueError('Invalid confirmed clearing geometry')
    if not bounds[0]<bounds[1] or not bounds[2]<bounds[3] or minimum_age<0.:
        raise ValueError('Invalid confirmed clearing bounds')
    result=deepcopy(data);removed=[];kept=[]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    for row in data['cells']:
        dx,dy=row[2]-pose[0],row[3]-pose[1]
        x,y=c*dx+s*dy,-s*dx+c*dy
        if (bounds[0]<=x<=bounds[1] and bounds[2]<=y<=bounds[3] and
                (allowed_hits is None or allowed_hits.get(tuple(row[:2]))==row[4]) and
                data['last_stamp']-row[4]>=minimum_age):removed.append(row)
        else:kept.append(row)
    result['cells']=kept;result['cleared']+=len(removed)
    keys={tuple(row[:2]) for row in removed}
    for field in ('dynamic_cells','first_seen','support_history','static_confirmed',
                  'untracked_landmarks','tracked_seen'):
        if field in result:result[field]=[row for row in result[field] if tuple(row[:2]) not in keys]
    return result,removed


def memory_checkpoint(memory,frame,session,dynamic):
    return {'version':1,'frame':frame,'session':session,'resolution':memory.resolution,
            'last_stamp':memory.last_stamp,'cleared':memory.cleared,
            'cells':[[*k,*v] for k,v in memory.cells.items()],
            'dynamic_cells':[[*k,v] for k,v in dynamic.dynamic_cells.items()],
            'first_seen':[[*k,v] for k,v in dynamic.first_seen.items()],
            'support_history':[[*k,*v] for k,v in dynamic.support_history.items()],
            'static_confirmed':[[*k,v] for k,v in dynamic.static_confirmed.items()],
            'untracked_landmarks':[[*k,v] for k,v in dynamic.untracked_landmarks.items()],
            'tracked_seen':[list(k) for k in dynamic.tracked_seen]}


def restore_checkpoint(data,memory,frame,session,dynamic):
    if (data.get('version')!=1 or data.get('frame')!=frame or data.get('session')!=session or
            data.get('resolution')!=memory.resolution):
        raise ValueError('Checkpoint is from a different LIO session/frame/resolution')
    if not math.isfinite(data['last_stamp']) or len(data['cells'])>memory.max_cells:
        raise ValueError('Invalid checkpoint bounds')
    cells={}
    for row in data['cells']:
        if len(row)!=9:raise ValueError('Invalid checkpoint cell')
        key=tuple(row[:2]);value=tuple(row[2:])
        if (any(not isinstance(k,int) for k in key) or key in cells or
            any(v is not None and not math.isfinite(v) for v in value) or
            value[2]>data['last_stamp'] or value[4]<0 or
            tuple(math.floor(v/memory.resolution) for v in value[:2])!=key):
            raise ValueError('Invalid checkpoint geometry')
        cells[key]=value
    memory.cells=cells;memory.last_stamp=data['last_stamp'];memory.cleared=int(data['cleared'])
    dynamic.dynamic_cells={tuple(v[:2]):int(v[2]) for v in data.get('dynamic_cells',[]) if tuple(v[:2]) in cells}
    dynamic.first_seen={tuple(v[:2]):float(v[2]) for v in data.get('first_seen',[]) if tuple(v[:2]) in cells}
    dynamic.next_id=max(dynamic.dynamic_cells.values(),default=0)+1
    dynamic.support_history={tuple(v[:2]):tuple(v[2:]) for v in data.get('support_history',[])
                             if tuple(v[:2]) in cells}
    if 'static_confirmed' in data:
        dynamic.static_confirmed={tuple(v[:2]):float(v[2]) for v in data['static_confirmed']
                                  if tuple(v[:2]) in cells}
    else:
        dynamic.static_confirmed={k:data['last_stamp']-1. for k in cells if k not in dynamic.dynamic_cells}
    dynamic.untracked_landmarks={tuple(v[:2]):float(v[2]) for v in data.get('untracked_landmarks',[])
                                if tuple(v[:2]) in cells}
    dynamic.tracked_seen={tuple(v) for v in data.get('tracked_seen',[]) if tuple(v) in cells}


def navigation_grid(data,width,height,source_resolution,target_resolution):
    """Refine the source conservatively before adding precise memory points.

    Any source cell overlapping an output cell contributes its cost; source
    occupied regions are never eroded by nearest-centre interpolation.
    """
    resolution=min(float(source_resolution),float(target_resolution))
    if not resolution>0.:raise ValueError('Invalid map resolution')
    source=np.asarray(data,dtype=np.int8).reshape(height,width)
    if abs(resolution-source_resolution)<1.e-8:return source.copy(),resolution
    ratio=source_resolution/resolution
    factor=int(round(ratio))
    if factor>=1 and abs(ratio-factor)<1.e-6:
        # Aligned 8 cm -> 4 cm needs exact replication, not four overlapping
        # sampling planes. Preserve unknowns and every occupied source cell.
        return np.repeat(np.repeat(source,factor,axis=0),factor,axis=1),resolution
    out_width=max(1,int(math.ceil(width*source_resolution/resolution-1.e-5)))
    out_height=max(1,int(math.ceil(height*source_resolution/resolution-1.e-5)))
    def indices(size,limit):
        lo=np.floor(np.arange(size)*resolution/source_resolution+1.e-9).astype(int)
        hi=np.floor(((np.arange(size)+1)*resolution)/source_resolution-1.e-9).astype(int)
        return np.clip(lo,0,limit-1),np.clip(hi,0,limit-1)
    x0,x1=indices(out_width,width);y0,y1=indices(out_height,height)
    cells=np.stack([source[y[:,None],x[None,:]] for y in (y0,y1) for x in (x0,x1)])
    grid=cells.max(axis=0)
    grid[(grid==0)&(cells<0).any(axis=0)]=-1
    return grid,resolution


def clearing_delta(geometry,rays,pose,stamp,previous_stamp,clear_window=4.,
                   max_gap=.60,clear_frames=8,clear_seconds=.5,radius=20.,ground_plane=None,
                   dynamic_flags=None,transient_flags=None):
    """Vectorized 3-D counter updates; return only changed rows, never geometry.

    Layout is the same seven columns stored in ObstacleMemory.cells. NaN
    encodes an absent confirmation timestamp or unknown measured height.
    """
    geometry=np.asarray(geometry,dtype=float).reshape(-1,7)
    empty=(np.empty(0,dtype=np.int64),np.empty((0,2)),np.empty(0,dtype=bool))
    if not len(geometry) or stamp<=previous_stamp:return empty
    free,support=rays.classify_many(geometry[:,:2],geometry[:,5],geometry[:,6])
    dynamic=np.zeros(len(geometry),dtype=bool)
    if ground_plane is not None:
        normal,offset=np.asarray(ground_plane[:3]),float(ground_plane[3])
        if not np.isfinite(ground_plane).all() or abs(np.linalg.norm(normal)-1.)>.001 or normal[2]<.9:
            raise ValueError('Invalid world ground plane')
        lower=geometry[:,:2]@normal[:2]+normal[2]*geometry[:,5]+offset
        upper=geometry[:,:2]@normal[:2]+normal[2]*geometry[:,6]+offset
        near=np.sum((geometry[:,:2]-pose[:2])**2,axis=1)<=2.**2
        # Only whole measured volumes within 6 cm of a supported local floor
        # qualify. A 10 cm box/leg remains occupied. Visibility or a strongly
        # measured current floor is needed before reclassifying old ground.
        candidates=np.flatnonzero((lower>=-.06)&(upper<=.06))
        if len(candidates):
            visible=(near[candidates]&rays.sees_beyond(geometry[candidates,:2]))|(
                rays.on_ground_many(geometry[candidates,:2],ground_plane))
            # Correct old *ground-type* records, even in the near-floor blind
            # patch, only with a strongly measured current local floor. A true
            # 10 cm leg's full recorded height can never satisfy this band.
            if rays.ground_supported(ground_plane):visible|=near[candidates]
            floor=candidates[visible]
            free[floor]=True;support[floor]=False
        if dynamic_flags is not None:
            dynamic=np.asarray(dynamic_flags,dtype=bool).reshape(len(geometry))
            selected=np.flatnonzero(dynamic)
            if len(selected):
                seen=rays.sees_beyond(geometry[selected,:2],ground_plane)
                free[selected]|=seen&~support[selected]
        if transient_flags is not None and rays.ground_supported(ground_plane):
            # Sparse pedestrian fragments may never form a compact track.
            # Short-lived groups still need eight real floor observations
            # beyond the old cells, no occupied support, and no occluder.
            # Established landmarks retain the ordinary 3-D volume test.
            transient=np.asarray(transient_flags,dtype=bool).reshape(len(geometry))
            selected=np.flatnonzero(transient&~dynamic&~support)
            if len(selected):
                free[selected]|=rays.sees_beyond(geometry[selected,:2],ground_plane,ground_only=True)
    since=geometry[:,3].copy();count=geometry[:,4].copy()
    reset=np.isfinite(since)&(stamp-since>clear_window)
    if stamp-previous_stamp>max_gap:reset[:]=True
    reset|=support
    since[reset]=np.nan;count[reset]=0
    since[free&~np.isfinite(since)]=stamp
    count[free]+=1
    frames=np.where(dynamic,3,clear_frames);seconds=np.where(dynamic,.35,clear_seconds)
    remove=(np.sum((geometry[:,:2]-pose[:2])**2,axis=1)>radius**2)|(
        free&(count>=frames)&(stamp-since>=seconds))
    unchanged_since=(since==geometry[:,3])|(np.isnan(since)&np.isnan(geometry[:,3]))
    changed=np.flatnonzero(remove|~unchanged_since|(count!=geometry[:,4]))
    return changed,np.column_stack((since[changed],count[changed])),remove[changed]


def reference_clear_volumes(cells, ground_plane, radius=.04):
    """Share proven removal with the reference view, preserving measured floor."""
    if ground_plane is None:return np.empty((0,5))
    cells=np.asarray(cells,dtype=float).reshape(-1,7)
    normal=np.asarray(ground_plane[:3]);offset=ground_plane[3]
    if normal[2]<.9:return np.empty((0,5))
    floor=-(cells[:,:2]@normal[:2]+offset)/normal[2]
    low=np.maximum(cells[:,5],floor+.08)
    valid=np.isfinite(low)&np.isfinite(cells[:,6])&(cells[:,6]>low)
    return np.column_stack((cells[valid,:2],low[valid],cells[valid,6],np.full(valid.sum(),radius)))


class RayEvidence:
    """Actual current 3-D returns, used to clear the old object's height volume."""
    def __init__(self, points, origin, resolution=.05):
        self.origin=np.asarray(origin,dtype=float)
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        distance=np.linalg.norm(points[:,:2]-self.origin[:2],axis=1)
        valid=(distance>.25)&(distance<=5.5)
        self.points=points[valid]
        self.tree=cKDTree(self.points[:,:2])
        self.distance=distance[valid]
        delta=self.points[:,:2]-self.origin[:2]
        self.angles=np.arctan2(delta[:,1],delta[:,0])
        self.resolution=resolution
        # Keep a narrow physical ray corridor: a beam passing beside a thin
        # chair leg cannot prove that the leg itself has disappeared.
        self.clearance_radius=.02
        self.angular_resolution=2*math.pi/1800
        order=np.argsort(self.angles)
        self.sorted_indices=np.tile(order,3)
        self.sorted_angles=np.concatenate((self.angles[order]-2*math.pi,
                                           self.angles[order],self.angles[order]+2*math.pi))

    def pairs(self,xy,radius):
        groups=self.tree.query_ball_point(xy,radius,return_sorted=False)
        counts=np.fromiter((len(g) for g in groups),dtype=np.int64,count=len(groups))
        if not counts.sum():return np.empty(0,dtype=int),np.empty(0,dtype=int)
        return np.repeat(np.arange(len(xy)),counts),np.concatenate(groups).astype(int)

    def bounds_many(self,xy):
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        low=np.full(len(xy),np.inf);high=np.full(len(xy),-np.inf)
        cells,points=self.pairs(xy,.04)
        np.minimum.at(low,cells,self.points[points,2]);np.maximum.at(high,cells,self.points[points,2])
        valid=np.isfinite(low);narrow=valid & (high-low<.05)
        mid=(low[narrow]+high[narrow])*.5
        low[narrow],high[narrow]=mid-.025,mid+.025
        return [(float(a),float(b)) if ok else None for a,b,ok in zip(low,high,valid)]

    def classify_many(self,xy,low,high):
        """Return independent free-ray and occupied-support evidence.

        Filter distance, height and angular visibility before expanding ray
        pairs. Unobserved cells stay in memory without an expensive tree query.
        """
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        low,high=np.asarray(low,dtype=float),np.asarray(high,dtype=float)
        count=len(xy);result=np.zeros(count,dtype=bool)
        supported=np.zeros(count,dtype=bool)
        if not count:return result,supported
        delta=xy-self.origin[:2];distance=np.linalg.norm(delta,axis=1)
        # The sensor's minimum *return* range is not a minimum ray crossing
        # range. A finite farther return also traverses the near segment.
        # Keep the actual height/occlusion tests below for that segment.
        eligible=np.flatnonzero((distance>self.clearance_radius)&(distance<=5.5)&
                               np.isfinite(low)&np.isfinite(high))
        if not len(eligible):return result,supported
        cells,points=self.pairs(xy[eligible],.025)
        cells=eligible[cells]
        support=(self.points[points,2]>low[cells]+.01)&(self.points[points,2]<high[cells]-.005)
        # Repeated indices all receive True; no atomic/reduction operation is
        # needed. Avoid a per-pair reduction when only setting membership bits.
        supported[cells[support]]=True
        eligible=eligible[~supported[eligible]]
        if not len(eligible):return result,supported
        angles=np.arctan2(delta[eligible,1],delta[eligible,0])
        half=np.arcsin(np.minimum(1.,self.clearance_radius/distance[eligible]))+1.e-10
        starts=np.searchsorted(self.sorted_angles,angles-half)
        ends=np.searchsorted(self.sorted_angles,angles+half,side='right')
        counts=ends-starts
        if not counts.sum():return result,supported
        cells=np.repeat(eligible,counts)
        offsets=np.arange(int(counts.sum()))-np.repeat(np.cumsum(counts)-counts,counts)
        points=self.sorted_indices[np.repeat(starts,counts)+offsets]
        difference=self.angles[points]-np.repeat(angles,counts)
        projected=distance[cells]*np.cos(difference)
        z=self.origin[2]+(self.points[points,2]-self.origin[2])*projected/self.distance[points]
        blocked=np.zeros(count,dtype=bool)
        shadow=(self.distance[points]<projected+.05)&(z>low[cells]-.01)&(z<high[cells]+.01)
        blocked[cells[shadow]]=True
        good=(projected>0.)&(self.distance[points]>projected+.20)&(z>low[cells]+.01)&(z<high[cells]-.01)
        result[cells[good]]=True
        return result&~blocked,supported

    def clears_many(self,xy,low,high):
        return self.classify_many(xy,low,high)[0]

    def sees_beyond(self,xy,ground_plane=None,ground_only=False):
        """Measured column visibility for already floor-consistent volumes."""
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        delta=xy-self.origin[:2];distance=np.linalg.norm(delta,axis=1)
        result=np.zeros(len(xy),dtype=bool);blocked=result.copy()
        eligible=np.flatnonzero((distance>self.clearance_radius)&(distance<=5.5))
        angles=np.arctan2(delta[eligible,1],delta[eligible,0])
        half=np.arcsin(np.minimum(1.,.02/distance[eligible]))+1.e-10
        start=np.searchsorted(self.sorted_angles,angles-half)
        end=np.searchsorted(self.sorted_angles,angles+half,side='right');counts=end-start
        if not counts.sum():return result
        cells=np.repeat(eligible,counts)
        offsets=np.arange(int(counts.sum()))-np.repeat(np.cumsum(counts)-counts,counts)
        points=self.sorted_indices[np.repeat(start,counts)+offsets]
        projected=distance[cells]*np.cos(self.angles[points]-np.repeat(angles,counts))
        beyond=self.distance[points]>projected+.20
        closer=self.distance[points]<=projected+.05
        if ground_plane is not None:
            # Floor endpoints do not occlude a tracked moving body's column.
            height=self.points[points]@np.asarray(ground_plane[:3])+ground_plane[3]
            closer&=height>.08
            if ground_only:beyond&=np.abs(height)<=.03
        elif ground_only:
            return result
        result[cells[beyond]]=True
        blocked[cells[closer]]=True
        return result&~blocked

    def ground_supported(self,plane):
        height=self.points@np.asarray(plane[:3])+plane[3]
        xy=self.points[np.abs(height)<.025,:2]
        return len(xy)>=40 and np.linalg.eigvalsh(np.cov(xy.T))[0]>.01

    def on_ground_many(self,xy,plane):
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        result=np.zeros(len(xy),dtype=bool)
        cells,points=self.pairs(xy,.04)
        height=self.points[points]@np.asarray(plane[:3])+plane[3]
        result[cells[np.abs(height)<=.03]]=True
        return result

    @staticmethod
    def bins(keys):
        order=np.argsort(keys)
        values,starts,counts=np.unique(keys[order],return_index=True,return_counts=True)
        return {int(v):order[s:s+n] for v,s,n in zip(values,starts,counts)}

    def nearby(self,x,y,radius=.04):
        return self.points[self.tree.query_ball_point((x,y),radius)]

    def bounds(self,x,y):
        points=self.nearby(x,y)
        if not len(points):return None
        low,high=float(points[:,2].min()),float(points[:,2].max())
        if high-low<.05:
            middle=.5*(high+low)
            low,high=middle-.025,middle+.025
        return low,high

    def clears(self,x,y,low,high):
        if low is None or high is None:return False
        support=self.nearby(x,y,radius=.025)
        if len(support) and np.any((support[:,2]>low+.01)&(support[:,2]<high-.005)):
            return False  # Raw returns still support this obstacle even if 2-D filtering missed it.
        dx,dy=x-self.origin[0],y-self.origin[1]
        distance=math.hypot(dx,dy)
        if not self.clearance_radius<distance<=5.5:return False
        angle=math.atan2(dy,dx)
        half=math.asin(min(1.,self.clearance_radius/distance))+1.e-10
        start=np.searchsorted(self.sorted_angles,angle-half)
        end=np.searchsorted(self.sorted_angles,angle+half,side='right')
        indices=self.sorted_indices[start:end]
        delta=self.angles[indices]-angle
        projected=distance*np.cos(delta)
        cross=distance*np.abs(np.sin(delta))
        z=self.origin[2]+(self.points[indices,2]-self.origin[2])*projected/self.distance[indices]
        if np.any((self.distance[indices]<projected+.05)&(z>low-.01)&(z<high+.01)):
            return False
        return bool(np.any((cross<=self.clearance_radius)&(projected>0.)&
                           (self.distance[indices]>projected+.20)&
                           (z>low+.01)&(z<high-.01)))


class ObstacleMemory:
    def __init__(self, resolution=.05, radius=20., clear_frames=8, clear_seconds=.5,
                 clear_margin=.20, max_cells=50000, require_3d_clearing=False,
                 max_observation_gap=.20, clear_window=4.):
        self.resolution, self.radius = resolution, radius
        self.clear_frames, self.clear_seconds = clear_frames, clear_seconds
        self.clear_margin, self.max_cells = clear_margin, max_cells
        self.require_3d_clearing=require_3d_clearing
        self.max_observation_gap=max_observation_gap
        # Only 3-D clearing accumulates intermittent rays. This is a bounded
        # confirmation window, never an obstacle lifetime or a free-space TTL.
        if not math.isfinite(clear_window) or clear_window<clear_seconds:
            raise ValueError('Clear evidence window must cover the minimum confirmation duration')
        self.clear_window=clear_window
        self.cells = {}
        self.last_stamp = -math.inf
        self.cleared = 0

    def mark(self, ranges, angle_min, increment, range_min, range_max, pose, stamp):
        """Fast occupied writes; clearing is computed separately from a snapshot."""
        if stamp<=self.last_stamp or increment<=0.:return False
        px,py,yaw=pose
        for i,distance in enumerate(ranges):
            if not math.isfinite(distance) or not range_min<=distance<=range_max:continue
            angle=angle_min+i*increment+yaw
            x,y=px+distance*math.cos(angle),py+distance*math.sin(angle)
            key=(math.floor(x/self.resolution),math.floor(y/self.resolution))
            old=self.cells.get(key)
            self.cells[key]=(x,y,stamp,None,0,*(old[5:] if old else (None,None)))
        if len(self.cells)>self.max_cells:
            raise RuntimeError('Obstacle memory capacity exceeded; stop instead of evicting near obstacles')
        self.last_stamp=stamp
        return True

    def mark_xyz(self,points,pose,stamp):
        """Remember real 3-D endpoints with height evidence in the same update."""
        if stamp<=self.last_stamp:return False
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        if len(points):
            keys=np.floor(points[:,:2]/self.resolution).astype(np.int64)
            distance=np.sum((points[:,:2]-pose[:2])**2,axis=1)
            order=np.lexsort((distance,keys[:,1],keys[:,0]));keys=keys[order];points=points[order]
            starts=np.r_[0,np.flatnonzero(np.any(np.diff(keys,axis=0),axis=1))+1]
            low=np.minimum.reduceat(points[:,2],starts);high=np.maximum.reduceat(points[:,2],starts)
            narrow=high-low<.05;middle=.5*(low[narrow]+high[narrow])
            low[narrow],high[narrow]=middle-.025,middle+.025
            # Convert in bulk: thousands of per-cell NumPy scalar indexing and
            # conversion operations consumed the ROS callback's scan budget.
            for key,point,a,b in zip(keys[starts].tolist(),points[starts].tolist(),
                                     low.tolist(),high.tolist()):
                key=tuple(key);old=self.cells.get(key)
                # A moving/partly occluded object often ends in one sparse
                # foot return. Do not replace its measured height volume with
                # that last 5 cm slice: farther rays can then never intersect
                # the slice and its occupancy becomes permanent. Accumulate
                # only continuous occupied observations, never across a gap
                # that could represent a different object in the same cell.
                if (old is not None and stamp-old[2]<=self.max_observation_gap and
                        old[5] is not None and old[6] is not None):
                    a,b=min(a,old[5]),max(b,old[6])
                self.cells[key]=(point[0],point[1],stamp,None,0,a,b)
        if len(self.cells)>self.max_cells:
            raise RuntimeError('Obstacle memory capacity exceeded; stop instead of evicting near obstacles')
        self.last_stamp=stamp
        return True

    def merge_clearing(self, snapshot, processed, stamp):
        """A delayed clearing result can never erase/revert a newer occupied hit."""
        removed=0
        for key,original in snapshot.items():
            current=self.cells.get(key)
            if current is None:continue
            result=processed.get(key)
            if current[2]>stamp:
                # The same occupied cell may still acquire its initial height
                # evidence. Never accept its old clear counter or position.
                if (current[5] is None and result is not None and result[5] is not None and
                    math.hypot(current[0]-result[0],current[1]-result[1])<=self.resolution):
                    self.cells[key]=(*current[:5],*result[5:])
                continue
            if current!=original:continue
            if result is None:
                del self.cells[key];removed+=1
            else:self.cells[key]=result
        self.cleared+=removed
        return removed

    def clearing_snapshot(self,origin,stamp):
        """Copy only old cells within the raw sensor's 3-D clearing envelope."""
        return {key:p for key,p in self.cells.items()
            if stamp-p[2]>=self.clear_seconds and p[5] is not None and
            (p[0]-origin[0])**2+(p[1]-origin[1])**2<=5.5**2}

    def merge_delta(self,snapshot,keys,indices,states,removed,stamp,cleared_cells=None):
        """Merge sparse worker counters only if the original cell is unchanged."""
        deleted=0
        for index,state,erase in zip(indices,states,removed):
            key=keys[int(index)];original=snapshot[key];current=self.cells.get(key)
            if current is None or current[2]>stamp or current!=original:continue
            if erase:
                del self.cells[key];deleted+=1
                if cleared_cells is not None:cleared_cells.append(original)
            else:
                since=float(state[0]) if math.isfinite(state[0]) else None
                self.cells[key]=(*current[:3],since,int(state[1]),*current[5:])
        self.cleared+=deleted
        return deleted

    def observe(self, ranges, angle_min, increment, range_min, range_max, pose, stamp, rays=None):
        if stamp <= self.last_stamp or increment <= 0.:
            return
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        remove = []
        old=list(self.cells.items())
        clear_evidence=supported=None
        if rays is not None and self.require_3d_clearing:
            clear_evidence,supported=rays.classify_many([(p[0],p[1]) for _,p in old],
                [p[5] for _,p in old],[p[6] for _,p in old])
        for cell_index,(key, cell) in enumerate(old):
            wx, wy, last_hit, since, count, low, high = cell
            if stamp-self.last_stamp>self.max_observation_gap:since,count=None,0
            if self.require_3d_clearing and since is not None and stamp-since>self.clear_window:
                since,count=None,0
            dx, dy = wx-px, wy-py
            if dx*dx+dy*dy > self.radius*self.radius:
                remove.append(key)
                continue
            evidence = False
            if self.require_3d_clearing:
                evidence=bool(clear_evidence[cell_index]) if clear_evidence is not None else False
            else:
                x, y = c*dx+s*dy, -s*dx+c*dy
                distance = math.hypot(x,y)
                index = int(round((math.atan2(y,x)-angle_min)/increment))
                if range_min <= distance <= range_max and 0 <= index < len(ranges):
                    measured = ranges[index]
                    # Infinity/NaN alone never prove the old volume is free.
                    evidence = (math.isfinite(measured) and range_min <= measured <= range_max and
                                measured > distance+self.clear_margin)
            if evidence:
                since = stamp if since is None else since
                count += 1
                if count >= self.clear_frames and stamp-since >= self.clear_seconds:
                    remove.append(key)
                    self.cleared += 1
                else:
                    self.cells[key] = (wx,wy,last_hit,since,count,low,high)
            else:
                # Sparse HAP rays may miss this narrow cell between two real
                # seen-through observations. Preserve evidence only inside its
                # window; an occupied return always cancels the whole sequence.
                if not self.require_3d_clearing or (supported is not None and supported[cell_index]):
                    since,count=None,0
                self.cells[key] = (wx,wy,last_hit,since,count,low,high)
        for key in remove:
            del self.cells[key]
        hits=[]
        for index, distance in enumerate(ranges):
            if not math.isfinite(distance) or not range_min <= distance <= range_max:
                continue
            angle = angle_min+index*increment+yaw
            wx, wy = px+distance*math.cos(angle), py+distance*math.sin(angle)
            hits.append((wx,wy))
        all_bounds=rays.bounds_many(hits) if rays is not None else [None]*len(hits)
        for (wx,wy),bounds in zip(hits,all_bounds):
            key = (math.floor(wx/self.resolution),math.floor(wy/self.resolution))
            previous=self.cells.get(key)
            low,high=bounds if bounds is not None else (previous[5:] if previous else (None,None))
            self.cells[key] = (wx,wy,stamp,None,0,low,high)
        if len(self.cells) > self.max_cells:
            raise RuntimeError('Obstacle memory capacity exceeded; stop instead of evicting near obstacles')
        self.last_stamp = stamp

    def points(self, pose=None, radius=None):
        if pose is None or radius is None:
            return [(p[0],p[1]) for p in self.cells.values()]
        return [(p[0],p[1]) for p in self.cells.values()
                if (p[0]-pose[0])**2+(p[1]-pose[1])**2 <= radius*radius]

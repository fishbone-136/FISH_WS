"""Bounded, metric 2.5-D reference-map construction (independent of ROS)."""

import numpy as np
from scipy.ndimage import uniform_filter
from scipy.spatial import cKDTree


def surface_slopes(elevation, resolution, baseline=.24):
    """Local least-squares planes, independent of adjacent step detection.

    Box sums compute each physical neighbourhood once per input grid, rather
    than solving a new point-cloud fit at every surface cell.
    """
    valid=np.isfinite(elevation)
    y,x=np.indices(elevation.shape,dtype=float);x*=resolution;y*=resolution
    z=np.where(valid,elevation,0.)
    size=2*max(1,int(np.ceil(baseline/resolution)))+1
    def total(values):
        return uniform_filter(values.astype(float),size=size,mode='constant')*size*size
    n=total(valid);safe=np.maximum(n,1.)
    sx,sy,sz=total(x*valid),total(y*valid),total(z)
    xx=total(x*x*valid)-sx*sx/safe;yy=total(y*y*valid)-sy*sy/safe
    xy=total(x*y*valid)-sx*sy/safe
    xz=total(x*z)-sx*sz/safe;yz=total(y*z)-sy*sz/safe
    determinant=xx*yy-xy*xy
    supported=(n>=6)&(np.minimum(xx,yy)/safe>=baseline*baseline/16.)&(determinant>1.e-12)
    denominator=np.where(supported,determinant,1.)
    return np.where(supported,np.degrees(np.arctan(np.hypot(
        (xz*yy-yz*xy)/denominator,(yz*xx-xz*xy)/denominator))),0.)


def cleared_point_mask(points, volumes, tree=None):
    """Re-use confirmed memory clearance on historical reference geometry.

    Volumes are x,y,z_min,z_max,radius in the same frame as the points. They
    contain measured free-space evidence, not a reference-point lifetime.
    """
    points=np.asarray(points).reshape(-1,3)
    volumes=np.asarray(volumes,dtype=float).reshape(-1,5)
    remove=np.zeros(len(points),dtype=bool)
    if not len(points) or not len(volumes):return remove
    if not np.isfinite(volumes).all() or (volumes[:,4]<=0).any():
        raise ValueError('Invalid clear volumes')
    tree=tree if tree is not None else cKDTree(points[:,:2])
    for volume,indices in zip(volumes,tree.query_ball_point(volumes[:,:2],volumes[:,4])):
        indices=np.asarray(indices,dtype=int)
        z=points[indices,2]
        remove[indices[(z>=volume[2])&(z<=volume[3])]]=True
    return remove


def voxel_filter(points, leaf):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    lower, upper = keys.min(axis=0), keys.max(axis=0)
    extent = [int(hi) - int(lo) + 1 for lo, hi in zip(lower, upper)]
    if extent[0] * extent[1] * extent[2] <= np.iinfo(np.int64).max:
        # A single integer voxel id avoids NumPy's much slower three-field
        # structured sort. Preserve the original first point and output order.
        shifted = keys - lower
        packed = (shifted[:, 0] * extent[1] + shifted[:, 1]) * extent[2] + shifted[:, 2]
        _, indices = np.unique(packed, return_index=True)
    else:
        # Keep equality exact even for an unusually large sparse extent.
        _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def pose_matrix(pose):
    """Convert [x, y, z, qx, qy, qz, qw] to a normalized rigid transform."""
    pose = np.asarray(pose, dtype=np.float64)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError('Invalid mapping pose')
    q = pose[3:]
    norm = np.linalg.norm(q)
    if norm < 1.e-8:
        raise ValueError('Invalid mapping quaternion')
    x, y, z, w = q / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    matrix[:3, 3] = pose[:3]
    return matrix


def transform_points(points, matrix):
    # Reference geometry is already float32; avoid a full float64 cloud and
    # conversion for every keyframe. Tracker/calibration matrices stay double.
    points = np.asarray(points, dtype=np.float32)
    matrix = np.asarray(matrix, dtype=np.float32)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def build_layers(points, resolution=0.15, min_points=2, max_extent=60.0,
                 max_slope=30.0, max_step=0.15, max_roughness=0.10,
                 obstacle_height=0.22, previous_bounds=None):
    """Return observed heights in metres and terrain costs; unknown stays NaN/-1.

    These persistent layers are reference maps. Unobserved rays are never
    filled as free, and no interpolation fabricates measured terrain.
    """
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return None
    if resolution <= 0 or min_points < 1 or max_extent < resolution:
        raise ValueError('Invalid mapping grid limits')
    lower = np.floor((points[:, :2].min(axis=0) - 0.3) / resolution) * resolution
    upper = np.ceil((points[:, :2].max(axis=0) + 0.3) / resolution) * resolution
    if previous_bounds is not None:
        # Retain grid bounds when a noisy extreme return moves across a cell.
        # Geometry/costs still rebuild, so a corrected pose leaves no obstacle ghost.
        lower = np.minimum(lower, np.asarray(previous_bounds[:2]))
        upper = np.maximum(upper, np.asarray(previous_bounds[2:]))
    lower = np.rint(lower / resolution) * resolution
    upper = np.rint(upper / resolution) * resolution
    span = upper - lower
    clipped = bool((span > max_extent).any())
    if clipped:
        # Bound allocation even if an upstream pose/cloud is corrupt.
        center = np.median(points[:, :2], axis=0)
        lower = np.maximum(lower, np.floor((center - max_extent / 2) / resolution) * resolution)
        upper = np.minimum(upper, lower + max_extent)
    width, height = np.maximum(1, np.ceil((upper - lower) / resolution - 1.e-6).astype(int))
    xy = np.floor((points[:, :2] - lower) / resolution).astype(int)
    inside = (xy[:, 0] >= 0) & (xy[:, 0] < width) & (xy[:, 1] >= 0) & (xy[:, 1] < height)
    xy = xy[inside]
    z = points[inside, 2].astype(np.float64)
    index = xy[:, 1] * width + xy[:, 0]
    size = int(width * height)
    count = np.bincount(index, minlength=size)
    sums = np.bincount(index, weights=z, minlength=size)
    squares = np.bincount(index, weights=z*z, minlength=size)
    minimum = np.full(size, np.inf)
    maximum = np.full(size, -np.inf)
    np.minimum.at(minimum, index, z)
    np.maximum.at(maximum, index, z)
    known = count >= min_points
    mean = sums / np.maximum(count, 1)
    roughness = np.sqrt(np.maximum(0.0, squares / np.maximum(count, 1) - mean*mean))
    elevation = np.where(known, minimum, np.nan).reshape(height, width)
    step = np.zeros((height, width))
    for dy, dx in [(0, 1), (1, 0), (1, 1), (1, -1)]:
        ya = slice(max(0, -dy), min(height, height - dy))
        xa = slice(max(0, -dx), min(width, width - dx))
        yb = slice(max(0, dy), min(height, height + dy))
        xb = slice(max(0, dx), min(width, width + dx))
        a = elevation[ya, xa]
        b = elevation[yb, xb]
        difference = np.where(np.isfinite(a) & np.isfinite(b), np.abs(a - b), 0.0)
        step[ya, xa] = np.maximum(step[ya, xa], difference)
        step[yb, xb] = np.maximum(step[yb, xb], difference)
    slope = surface_slopes(elevation,resolution).ravel()
    normalized = np.maximum.reduce([
        slope / max_slope, step.ravel() / max_step, roughness / max_roughness,
    ])
    normalized[(maximum - minimum) >= obstacle_height] = 1.0
    costs = np.full(size, -1, dtype=np.int8)
    costs[known] = np.where(normalized[known] >= 1.0, 100,
                            np.rint(normalized[known] * 90)).astype(np.int8)
    ys, xs = np.nonzero(known.reshape(height, width))
    surface = np.column_stack([
        lower[0] + (xs + 0.5) * resolution,
        lower[1] + (ys + 0.5) * resolution,
        elevation[ys, xs],
    ]).astype(np.float32)
    return dict(origin=lower, width=int(width), height=int(height), resolution=resolution,
                elevation=elevation.astype(np.float32), costs=costs.reshape(height, width),
                surface=surface, surface_costs=costs[known].astype(np.float32), clipped=clipped)

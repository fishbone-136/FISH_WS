"""Factory Unitree point-cloud format and checked mounting geometry."""

import math

import numpy as np


ROTATION = np.array([[.5261139467, -.8101357845, .2586196546],
                     [-.8386681429, -.5446427692, .0000015836],
                     [.1408540419, -.2168968986, -.9659792308]])
TRANSLATION = np.array([.2821600512, 0., 0.])


def decode_cloud(message):
    """Decode only the verified factory format, including correspondence IDs."""
    fields = {f.name: f for f in message.fields}
    expected = {'x': (7, 0), 'y': (7, 4), 'z': (7, 8),
                'ring': (4, 20), 'time': (7, 24)}
    if (message.height != 1 or message.point_step != 32 or
            message.row_step < message.width * 32 or
            any(k not in fields or fields[k].datatype != kind or
                fields[k].offset != offset or fields[k].count != 1
                for k, (kind, offset) in expected.items())):
        raise ValueError('Unsupported factory cloud format')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z', 'ring', 'time_bits'],
        'formats': [endian+'f4']*3+[endian+'u2', endian+'u4'],
        'offsets': [0, 4, 8, 20, 24], 'itemsize': 32})
    a = np.frombuffer(message.data, dtype=dtype, count=message.width)
    points = np.column_stack([a[k] for k in ('x', 'y', 'z')])
    keys = (a['ring'].astype(np.uint64) << 32) | a['time_bits'].astype(np.uint64)
    return points, keys


def factory_transform_error(raw, body):
    _, raw_indices, body_indices = np.intersect1d(raw[1], body[1], return_indices=True)
    source, target = raw[0][raw_indices], body[0][body_indices]
    valid = (np.isfinite(source).all(1) & np.isfinite(target).all(1) &
             (np.linalg.norm(source, axis=1) > .01))
    source, target = source[valid], target[valid]
    if len(source) < 32:
        return math.inf
    sample = np.linspace(0, len(source)-1, min(128, len(source)), dtype=int)
    return float(np.max(np.linalg.norm(
        source[sample] @ ROTATION.T + TRANSLATION - target[sample], axis=1)))

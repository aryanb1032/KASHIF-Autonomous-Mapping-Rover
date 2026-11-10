"""Select a known-free, reachable approach to an unknown map boundary."""
import math
import numpy as np
from scipy import ndimage


def candidates(data, resolution, robot_cell, clearance=0.30):
    grid = np.asarray(data)
    free = (grid >= 0) & (grid <= 20)
    obstacles = grid >= 50
    # Pad as occupied so goals never hang outside the supplied map.
    padded = np.pad(~obstacles, 1, constant_values=False)
    distance = ndimage.distance_transform_edt(padded)[1:-1,1:-1] * resolution
    safe = free & (distance > clearance)
    labels, _ = ndimage.label(safe)
    y,x = robot_cell
    if not (0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]): return []
    region = labels[y,x]
    if not region: return []
    reachable = labels == region
    unknown_labels, unknown_count = ndimage.label(grid < 0)
    sizes = np.bincount(unknown_labels.ravel())
    large = sizes >= max(40, math.ceil(0.25 / resolution**2))
    large[0] = False
    unknown = large[unknown_labels]
    border = free & ndimage.binary_dilation(unknown)
    front_labels, count = ndimage.label(border)
    result=[]
    for label in range(1,count+1):
        frontier = front_labels == label
        size = int(frontier.sum())
        if size < 3: continue
        d = ndimage.distance_transform_edt(~frontier)*resolution
        approach = reachable & (d >= 0.30) & (d <= 0.65)
        ys,xs = np.where(approach)
        if not len(xs): continue
        travel = np.hypot(xs-x,ys-y)*resolution
        valid = travel > 0.45
        if not valid.any(): continue
        ys,xs,travel = ys[valid],xs[valid],travel[valid]
        idx = int(np.argmin(travel))
        result.append((float(travel[idx]) / math.sqrt(size),int(ys[idx]),int(xs[idx])))
    return sorted(result)

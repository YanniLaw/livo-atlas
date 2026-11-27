"""Deterministic image-grid sampling with exact capacity-limited quotas."""

import math

import numpy as np


def stratified_pixels(pixels, count, shape):
    """Give nonempty image cells equal quotas, redistributing saturated cells."""
    pixels = np.asarray(pixels)
    if (len(shape) != 2 or any(type(v) is not int or v <= 0 for v in shape) or
            shape[0]*shape[1] > 1_000_000 or pixels.ndim != 1 or pixels.dtype.kind not in 'iu' or
            type(count) is not int or not 0 <= count <= len(pixels) or
            (len(pixels) and (pixels[0] < 0 or pixels[-1] >= shape[0]*shape[1] or np.any(pixels[1:] <= pixels[:-1])))):
        raise ValueError('Invalid stratified pixel list, budget or image shape')
    pixels = pixels.astype('i8', copy=False)
    if not count or count == len(pixels):
        return pixels[:count].copy()
    height, width = shape
    cols = min(width, count, max(1, round(math.sqrt(count*width/height))))
    rows = min(height, (count+cols-1)//cols)
    y, x = np.divmod(pixels, width)
    cells = (y*rows//height)*cols + x*cols//width
    order = np.argsort(cells, kind='stable')
    _, capacity = np.unique(cells[order], return_counts=True)
    low, high = 0, int(capacity.max())+1
    while low+1 < high:
        level = (low+high)//2
        if int(np.minimum(capacity, level).sum()) <= count:
            low = level
        else:
            high = level
    quota = np.minimum(capacity, low)
    remainder = count-int(quota.sum())
    if remainder:
        active = np.flatnonzero(capacity > low)
        quota[active[np.arange(remainder)*len(active)//remainder]] += 1
    starts = np.concatenate(([0], np.cumsum(capacity)[:-1]))
    selected = []
    for start, size, n in zip(starts, capacity, quota):
        if n:
            # Centered raster quantiles inside each cell are deterministic and
            # unique. Cell capacity is the only restriction on redistribution.
            indices = start+(2*np.arange(n)+1)*size//(2*n)
            selected.append(pixels[order[indices]])
    return np.sort(np.concatenate(selected))

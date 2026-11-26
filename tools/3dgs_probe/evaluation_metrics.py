"""Fixed heldout image metrics and reproducible sufficient statistics (CPU)."""

import math

import numpy as np

from joint_renderer import _ALPHA_THRESHOLD


def _validated(result, query):
    rgb = np.asarray(query['rgb'])
    if rgb.ndim != 3 or rgb.shape[2] != 3 or not 0 < rgb.shape[0]*rgb.shape[1] <= 1_000_000:
        raise ValueError('Evaluation reference exceeds the image budget')
    shape = rgb.shape[:2]
    for source, key, expected in ((query, 'rgb', rgb.shape), (result, 'rgb', rgb.shape),
                                 (result, 'alpha', shape), (result, 'depth_m', shape),
                                 (query, 'depth_m', shape)):
        value = np.asarray(source[key])
        if value.shape != expected or value.dtype.kind != 'f' or not np.isfinite(value).all():
            raise ValueError('Invalid evaluation array: ' + key)
        if np.any(value < 0) or (key != 'depth_m' and np.any(value > 1 + 1e-6)):
            raise ValueError('Evaluation values are outside their range: ' + key)
    masks = []
    for key in ('image_mask', 'depth_mask'):
        value = np.asarray(query[key])
        if value.shape != shape or value.dtype not in (np.dtype('bool'), np.dtype('u1')) or np.any(value > 1):
            raise ValueError('Invalid evaluation mask: ' + key)
        masks.append(value.astype(bool, copy=False))
    if np.any(masks[1] & ~masks[0]) or np.any(query['depth_m'][masks[1]] <= 0):
        raise ValueError('Evaluation depth labels disagree with masks')
    return masks


def _gaussian(values):
    # Separable valid convolution avoids a H*W*121 temporary window array.
    axis = np.arange(-5, 6, dtype='f8')
    weights = np.exp(-axis**2 / (2 * 1.5**2))
    weights /= weights.sum()
    h, w = values.shape
    horizontal = np.zeros((h, w-10), 'f8')
    for index, weight in enumerate(weights):
        horizontal += weight * values[:, index:index+w-10]
    result = np.zeros((h-10, w-10), 'f8')
    for index, weight in enumerate(weights):
        result += weight * horizontal[index:index+h-10]
    return result


def _ssim(reference, rendered, mask):
    if min(mask.shape) < 11:
        return 0.0, 0, 'image_smaller_than_window'
    integral = np.pad(mask.astype('i8'), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    counts = integral[11:, 11:] - integral[:-11, 11:] - integral[11:, :-11] + integral[:-11, :-11]
    valid = counts == 121
    count = int(valid.sum())
    if not count:
        return 0.0, 0, 'no_complete_valid_window'
    total = 0.0
    for channel in range(3):
        a, b = reference[..., channel].astype('f8'), rendered[..., channel].astype('f8')
        ma, mb = _gaussian(a), _gaussian(b)
        va = np.maximum(_gaussian(a*a) - ma*ma, 0)
        vb = np.maximum(_gaussian(b*b) - mb*mb, 0)
        cov = _gaussian(a*b) - ma*mb
        values = ((2*ma*mb + .01**2) * (2*cov + .03**2)) / (
            (ma*ma + mb*mb + .01**2) * (va + vb + .03**2))
        total += float(np.clip(values[valid], -1, 1).sum()) / 3
    return total, count, None


def _derived(stats):
    image, depth, windows = (stats[key] for key in ('image_pixels', 'depth_pixels_used', 'ssim_windows'))
    mse = stats['rgb_squared_error_sum'] / (3*image) if image else None
    available = stats['depth_pixels_available']
    result = dict(stats)
    result.update(rgb_mse=mse, rgb_perfect=mse == 0 if mse is not None else False,
                  rgb_psnr_db=-10*math.log10(mse) if mse is not None and mse > 0 else None,
                  rgb_l1=stats['rgb_absolute_error_sum']/(3*image) if image else None,
                  alpha_coverage=stats['covered_pixels']/image if image else None,
                  depth_l1_m=stats['depth_absolute_error_sum']/depth if depth else None,
                  depth_rmse_m=math.sqrt(stats['depth_squared_error_sum']/depth) if depth else None,
                  depth_coverage=depth/available if available else None,
                  depth_reason=None if depth else ('no_reference_depth' if not available
                                                  else 'no_covered_reference_depth'),
                  ssim_rgb=stats['ssim_sum']/windows if windows else None)
    return result


def measure(result, query):
    """RGB includes holes; depth is camera-z consistency on covered labels."""
    image, labels = _validated(result, query)
    covered = image & (result['alpha'] >= _ALPHA_THRESHOLD)
    depth = labels & covered
    error = result['rgb'][image].astype('f8') - query['rgb'][image].astype('f8')
    z_error = result['depth_m'][depth].astype('f8') - query['depth_m'][depth].astype('f8')
    ssim_sum, windows, reason = _ssim(query['rgb'], result['rgb'], image)
    return _derived(dict(image_pixels=int(image.sum()), covered_pixels=int(covered.sum()),
                         alpha_threshold=_ALPHA_THRESHOLD,
                         rgb_squared_error_sum=float(np.square(error).sum()),
                         rgb_absolute_error_sum=float(np.abs(error).sum()),
                         depth_pixels_available=int(labels.sum()), depth_pixels_used=int(depth.sum()),
                         depth_squared_error_sum=float(np.square(z_error).sum()),
                         depth_absolute_error_sum=float(np.abs(z_error).sum()),
                         ssim_windows=windows, ssim_sum=ssim_sum, ssim_reason=reason))


def aggregate_metrics(rows):
    if not rows:
        return None
    keys = ('image_pixels', 'covered_pixels', 'rgb_squared_error_sum', 'rgb_absolute_error_sum',
            'depth_pixels_available', 'depth_pixels_used', 'depth_squared_error_sum',
            'depth_absolute_error_sum', 'ssim_windows', 'ssim_sum')
    totals = {key: sum(row[key] for row in rows) for key in keys}
    totals.update(alpha_threshold=_ALPHA_THRESHOLD,
                  ssim_reason=None if totals['ssim_windows'] else 'no_complete_valid_window')
    return _derived(totals)

"""CPU texture diagnostics on fixed coverage and reference-only gradient cohorts."""

import math

import numpy as np

from evaluation_metrics import _gaussian
from joint_renderer import _ALPHA_THRESHOLD


GRADIENT_THRESHOLD = .02
COHORTS = ('all', 'weak', 'strong', 'unknown')
STATES = ('common_covered', 'gained', 'lost', 'common_holes')


def _rgb(value, shape=None):
    value = np.asarray(value)
    if (value.ndim != 3 or value.shape[2] != 3 or not 0 < value.shape[0]*value.shape[1] <= 1_000_000
            or (shape is not None and value.shape != shape) or value.dtype.kind != 'f'
            or not np.isfinite(value).all() or np.any(value < 0) or np.any(value > 1+1e-6)):
        raise ValueError('Invalid texture RGB array')
    return value


def _mask(value, shape):
    value = np.asarray(value)
    if value.shape != shape or value.dtype not in (np.dtype('bool'), np.dtype('u1')) or np.any(value > 1):
        raise ValueError('Invalid texture mask')
    return value.astype(bool, copy=False)


def gradient_masks(rgb, image_mask):
    """Four-neighbor RGB difference energy; boundary or missing neighbors are unknown."""
    rgb = _rgb(rgb).astype('f8')
    mask = _mask(image_mask, rgb.shape[:2])
    known = np.zeros_like(mask)
    known[1:-1, 1:-1] = (mask[1:-1, 1:-1] & mask[:-2, 1:-1] & mask[2:, 1:-1]
                         & mask[1:-1, :-2] & mask[1:-1, 2:])
    energy = np.zeros(mask[1:-1, 1:-1].shape, 'f8')
    center = rgb[1:-1, 1:-1]
    for neighbor in (rgb[:-2, 1:-1], rgb[2:, 1:-1], rgb[1:-1, :-2], rgb[1:-1, 2:]):
        energy += np.square(neighbor-center).sum(axis=2)/6
    strong = np.zeros_like(mask)
    strong[1:-1, 1:-1] = np.sqrt(energy) >= GRADIENT_THRESHOLD
    strong &= known
    return dict(strong=strong, weak=known & ~strong, unknown=mask & ~known)


def _derived(stats):
    result = dict(stats)
    pixels = result['pixels']
    mse = result['rgb_squared_sum']/(3*pixels) if pixels else None
    result.update(rgb_mse=mse, rgb_l1=result['rgb_absolute_sum']/(3*pixels) if pixels else None,
                  rgb_psnr_db=-10*math.log10(mse) if mse is not None and mse > 0 else None,
                  rgb_perfect=mse == 0 if pixels else False,
                  rgb_reason='no_selected_pixels' if not pixels else ('perfect_rgb' if mse == 0 else None))
    if 'ssim_windows' in result:
        count = result['ssim_windows']
        result['ssim_rgb'] = result['ssim_sum']/count if count else None
        if count:
            result['ssim_reason'] = None
    return result


def _error_stats(reference, rgb, mask):
    error = rgb[mask].astype('f8') - reference[mask].astype('f8')
    return _derived(dict(pixels=int(mask.sum()), rgb_absolute_sum=float(np.abs(error).sum()),
                         rgb_squared_sum=float(np.square(error).sum())))


def _complete_windows(mask):
    integral = np.pad(mask.astype('i8'), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    counts = integral[11:, 11:] - integral[:-11, 11:] - integral[11:, :-11] + integral[:-11, :-11]
    return counts == 121


def _ssim_channels(reference, rgb):
    # Keep the established evaluator's arithmetic/accumulation order for exact all-region parity.
    for channel in range(3):
        a, b = reference[..., channel].astype('f8'), rgb[..., channel].astype('f8')
        ma, mb = _gaussian(a), _gaussian(b)
        va = np.maximum(_gaussian(a*a)-ma*ma, 0)
        vb = np.maximum(_gaussian(b*b)-mb*mb, 0)
        cov = _gaussian(a*b)-ma*mb
        yield ((2*ma*mb+.01**2)*(2*cov+.03**2))/((ma*ma+mb*mb+.01**2)*(va+vb+.03**2))


def texture_metrics(reference, rgb, regions, gradients):
    reference = _rgb(reference)
    rgb = _rgb(rgb, reference.shape)
    shape = reference.shape[:2]
    if set(regions) != {'image', 'common'} or set(gradients) != {'weak', 'strong', 'unknown'}:
        raise ValueError('Expected image/common regions and three gradient cohorts')
    regions = {name: _mask(mask, shape) for name, mask in regions.items()}
    gradients = {name: _mask(mask, shape) for name, mask in gradients.items()}
    if (np.any(regions['common'] & ~regions['image']) or
            not np.array_equal(sum(mask.astype('u1') for mask in gradients.values()), regions['image'].astype('u1'))):
        raise ValueError('Regions or gradient cohorts do not partition the image')
    small = min(shape) < 11
    results, windows = {}, {}
    for region, mask in regions.items():
        results[region] = {}
        valid = None if small else _complete_windows(mask)
        for cohort in COHORTS:
            selected = mask if cohort == 'all' else mask & gradients[cohort]
            row = _error_stats(reference, rgb, selected)
            centers = None if small else valid & (True if cohort == 'all' else gradients[cohort][5:-5, 5:-5])
            count = 0 if small else int(centers.sum())
            row.update(ssim_windows=count, ssim_sum=0., ssim_reason=None if count else
                       ('image_smaller_than_window' if small else 'no_complete_valid_window'))
            windows[region, cohort] = centers
            results[region][cohort] = row
    if not small:
        for values in _ssim_channels(reference, rgb):
            for (region, cohort), mask in windows.items():
                results[region][cohort]['ssim_sum'] += float(np.clip(values[mask], -1, 1).sum())/3
    return {region: {cohort: _derived(row) for cohort, row in rows.items()} for region, rows in results.items()}


def aggregate_stats(rows):
    if not rows:
        raise ValueError('Cannot pool an empty statistic list')
    keys = ['pixels', 'rgb_absolute_sum', 'rgb_squared_sum']
    texture = 'ssim_windows' in rows[0]
    if any(('ssim_windows' in row) != texture for row in rows):
        raise ValueError('Cannot mix RGB and texture statistics')
    if texture:
        keys += ['ssim_windows', 'ssim_sum']
    result = {key: sum(row[key] for row in rows) for key in keys}
    if texture:
        reasons = {row['ssim_reason'] for row in rows}
        result['ssim_reason'] = next(iter(reasons)) if len(reasons) == 1 else 'no_complete_valid_window'
    return _derived(result)


def _partition_result(states, total):
    pixels = total['baseline']['pixels']
    for row in states.values():
        for field, key in (('mse', 'rgb_squared_sum'), ('l1', 'rgb_absolute_sum')):
            row[field+'_delta_contribution'] = (row['candidate'][key]-row['baseline'][key])/(3*pixels) if pixels else None
    residuals = {label: {key: total[label][key]-sum(row[label][key] for row in states.values())
                          for key in ('pixels', 'rgb_absolute_sum', 'rgb_squared_sum')}
                 for label in ('baseline', 'candidate')}
    for label, residual in residuals.items():
        if any(abs(value) > 1e-10*max(1, abs(total[label][key])) for key, value in residual.items()):
            raise ValueError('Four coverage states do not reconstruct image statistics')
    return dict(image_pixels=pixels, states=states, residuals=residuals, **{'global': total})


def coverage_partition(reference, baseline, candidate, image_mask):
    reference = _rgb(reference)
    mask = _mask(image_mask, reference.shape[:2])
    covered = []
    for rendered in (baseline, candidate):
        _rgb(rendered['rgb'], reference.shape)
        alpha = np.asarray(rendered['alpha'])
        if (alpha.shape != mask.shape or alpha.dtype.kind != 'f' or not np.isfinite(alpha).all()
                or np.any(alpha < 0) or np.any(alpha > 1+1e-6)):
            raise ValueError('Invalid texture alpha array')
        covered.append(alpha >= _ALPHA_THRESHOLD)
    a, b = covered
    masks = (mask & a & b, mask & ~a & b, mask & a & ~b, mask & ~a & ~b)
    states = {name: {label: _error_stats(reference, value['rgb'], selected)
                     for label, value in (('baseline', baseline), ('candidate', candidate))}
              for name, selected in zip(STATES, masks)}
    total = {label: _error_stats(reference, value['rgb'], mask)
             for label, value in (('baseline', baseline), ('candidate', candidate))}
    return _partition_result(states, total)


def aggregate_partitions(rows):
    if not rows:
        raise ValueError('Cannot pool an empty partition list')
    states = {state: {label: aggregate_stats([row['states'][state][label] for row in rows])
                      for label in ('baseline', 'candidate')} for state in STATES}
    total = {label: aggregate_stats([row['global'][label] for row in rows]) for label in ('baseline', 'candidate')}
    return _partition_result(states, total)

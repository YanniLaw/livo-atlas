"""Bounded pixel-neighborhood center support; this is not a Gaussian footprint."""

import numpy as np

from bundle import _pose


RADII = (0, 2, 4)
COHORTS = ('all_initial', 'retained_initial', 'retained_trained')
FIELDS = ('any', 'consistent', 'front', 'behind')


def _query_masks(query):
    image, labels, depth = (np.asarray(query[key]) for key in ('image_mask','depth_mask','depth_m'))
    if (image.ndim != 2 or not 0 < image.size <= 1_000_000 or labels.shape != image.shape or
            depth.shape != image.shape or depth.dtype.kind != 'f' or not np.isfinite(depth).all()):
        raise ValueError('Invalid coverage query dimensions or depth')
    for mask in (image, labels):
        if mask.dtype not in (np.dtype(bool), np.dtype('u1')) or np.any(mask > 1):
            raise ValueError('Invalid coverage query mask')
    image, labels = image.astype(bool), labels.astype(bool)
    if np.any(labels & ~image) or np.any(depth[labels] <= 0):
        raise ValueError('Coverage depth labels disagree with image mask')
    return image, labels, depth


def projected_support(points, camera_from_points, K, query, radius):
    image, labels, depth = _query_masks(query)
    points, k = np.asarray(points), np.asarray(K, dtype='f8')
    camera = _pose(camera_from_points)
    if (type(radius) is not int or radius not in RADII or points.ndim != 2 or points.shape[1] != 3 or
            len(points) > 64000 or points.dtype.kind != 'f' or not np.isfinite(points).all()):
        raise ValueError('Invalid bounded points or pixel radius')
    if (k.shape != (3,3) or not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0 or
            not np.allclose(k[2], [0,0,1], atol=1e-12, rtol=0) or
            abs(k[0,1]) > 1e-12 or abs(k[1,0]) > 1e-12):
        raise ValueError('Invalid pinhole intrinsics')
    height, width = image.shape
    xyz = points.astype('f8') @ camera[:3,:3].T + camera[:3,3]
    xyz = xyz[xyz[:,2] > 1e-6]
    u = k[0,0]*xyz[:,0]/xyz[:,2] + k[0,2] + .5
    v = k[1,1]*xyz[:,1]/xyz[:,2] + k[1,2] + .5
    # Keep just-outside centers when their neighborhood reaches the image.
    keep = np.isfinite(u) & np.isfinite(v) & (u >= -radius) & (u < width+radius) & (v >= -radius) & (v < height+radius)
    xx, yy, z = np.floor(u[keep]).astype('i8'), np.floor(v[keep]).astype('i8'), xyz[keep,2]
    result = {key:np.zeros(image.shape,dtype='i4') for key in FIELDS}
    for dy in range(-radius,radius+1):
        for dx in range(-radius,radius+1):
            if dx*dx+dy*dy > radius*radius:
                continue
            x, y = xx+dx, yy+dy
            ids = np.flatnonzero((x >= 0) & (x < width) & (y >= 0) & (y < height))
            ids = ids[image[y[ids],x[ids]]]
            np.add.at(result['any'], (y[ids],x[ids]), 1)
            ids = ids[labels[y[ids],x[ids]]]
            observed = depth[y[ids],x[ids]]
            error, tolerance = z[ids]-observed, np.maximum(.05,.02*observed)
            for name, match in (('consistent',np.abs(error) <= tolerance), ('front',error < -tolerance), ('behind',error > tolerance)):
                selected = ids[match]
                np.add.at(result[name], (y[selected],x[selected]), 1)
    return result


def coverage_summary(query, initial, trained, support):
    image, labels, _ = _query_masks(query)
    alphas = [np.asarray(row['alpha']) for row in (initial,trained)]
    if any(a.shape != image.shape or a.dtype.kind != 'f' or not np.isfinite(a).all() or
           np.any(a < 0) or np.any(a > 1+1e-6) for a in alphas):
        raise ValueError('Invalid diagnostic alpha')
    old, new = (a >= .05 for a in alphas)
    holes, known = image & ~new, labels & ~new
    count = lambda mask:int(mask.sum())
    result = dict(image_pixels=count(image), reference_pixels=count(labels),
        initial_covered=count(image & old), trained_covered=count(image & new),
        common_covered=count(image & old & new), persistent_holes=count(image & ~old & ~new),
        gained_coverage=count(image & ~old & new), lost_coverage=count(image & old & ~new),
        hole_depth_known=count(known), hole_depth_unknown=count(holes & ~labels), support={}, retention_proxy={})
    for cohort in COHORTS:
        result['support'][cohort] = {}
        for radius in RADII:
            maps = support[cohort][radius]
            if any(v.shape != image.shape or v.dtype.kind not in 'iu' or np.any(v < 0) for v in maps.values()):
                raise ValueError('Invalid center-support counts')
            if np.any((maps['consistent']+maps['front']+maps['behind'])[labels] != maps['any'][labels]):
                raise ValueError('Depth classes do not partition labelled support')
            c, f, b, any_point = (maps[key] > 0 for key in ('consistent','front','behind','any'))
            classes = dict(consistent=count(known & c), front_only=count(known & ~c & f & ~b),
                behind_only=count(known & ~c & b & ~f), mixed=count(known & ~c & f & b), none=count(known & ~any_point))
            if sum(classes.values()) != count(known):
                raise ValueError('Hole classes do not partition known holes')
            result['support'][cohort][str(radius)] = dict(holes_near_centers=count(holes & any_point),
                image_pixels_near_centers=count(image & any_point), labelled_holes=classes)
    for radius in RADII:
        a, b = support['all_initial'][radius], support['retained_initial'][radius]
        if any(np.any(a[key] < b[key]) for key in FIELDS):
            raise ValueError('Retained initial centers are not a subset')
        result['retention_proxy'][str(radius)] = dict(
            holes_losing_initial_neighbors=count(holes & (a['any'] > 0) & (b['any'] == 0)),
            known_holes_losing_consistent_initial_neighbors=count(known & (a['consistent'] > 0) & (b['consistent'] == 0)))
    return result

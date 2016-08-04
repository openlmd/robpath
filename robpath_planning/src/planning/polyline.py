import numpy as np

"""Ramer-Douglas-Peucker line simplification demo."""


def _vec_dist(p1, p2):
    return np.sum(np.square(p1 - p2))


def _vec_sub(p1, p2):
    return p1 - p2


def _vec_mult(p1, p2):
    return np.sum(p1 * p2)


def ramerdouglas(line, dist):
    """Does Ramer-Douglas-Peucker simplification of a curve with `dist`
    threshold.

    Usage is like so:

    >>> myline = np.array([(0.0, 0.0), (1.0, 2.0), (2.0, 1.0)])
    >>> simplified = ramerdouglas(myline, dist=1.0)
    """

    if len(line) < 3:
        return line

    (begin, end) = (line[0], line[-1]) if np.any(np.not_equal(line[0], line[-1])) else (line[0], line[-2])

    distSq = []
    for curr in line[1:-1]:
        tmp = (
            _vec_dist(begin, curr) - _vec_mult(_vec_sub(end, begin), _vec_sub(curr, begin)) ** 2 / _vec_dist(begin, end))
        distSq.append(tmp)

    maxdist = max(distSq)
    if maxdist < dist ** 2:
        return [begin, end]

    pos = distSq.index(maxdist)
    return (list(ramerdouglas(line[:pos + 2], dist)) +
            list(ramerdouglas(line[pos + 1:], dist))[1:])


def filter_polyline(points, dist=0.1):
    points = np.array(ramerdouglas(points, dist))
    return points


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    points = np.array([(6.247872, 11.316756),
                       (6.610650, 9.288978),
                       (6.542629, 9.066756),
                       (6.565303, 8.900089),
                       (6.542629, 8.761200),
                       (8.900686, 7.261200),
                       (8.809991, 7.261200),
                       (8.472735, 7.171122),
                       (8.333845, 7.038978),
                       (8.282022, 6.981100),
                       (8.254778, 6.848911),
                       (8.265824, 6.816756),
                       (8.239206, 6.711211),
                       (8.219743, 6.612067),
                       (8.130227, 6.433044),
                       (8.084435, 6.316756),
                       (8.107109, 6.288978),
                       (7.948394, 6.177867),
                       (7.925720, 5.983422),
                       (7.857699, 5.816756),
                       (7.835026, 5.788978),
                       (7.630963, 4.261200),
                       (7.540268, 4.177867),
                       (7.494921, 4.150089),
                       (7.449574, 4.150089),
                       (7.404227, 4.150089),
                       (7.336206, 4.094533),
                       (7.313532, 4.066756),
                       (7.041449, 4.011200),
                       (6.905407, 3.955644),
                       (6.950754, 3.900089),
                       (8.900686, 3.150089),
                       (8.787318, 2.900089),
                       (7.812352, 1.788978),
                       (7.767005, 1.538978),
                       (7.676310, 1.622311),
                       (7.653637, 1.650089),
                       (7.585616, 1.955644),
                       (7.562942, 1.983422),
                       (7.562942, 2.233422),
                       (7.608289, 2.400089),
                       (7.630963, 2.427867),
                       (5.998462, 5.233422),
                       (6.225198, 5.233422),
                       (6.270545, 5.233422),
                       (6.383914, 5.288978),
                       (6.406587, 5.372311),
                       (6.429261, 5.400089)])

    points = np.hstack((points, np.random.random_sample((len(points), 1))))
    pnts = np.array(ramerdouglas(points, 0.1))
    print len(points), len(pnts)

    plt.figure()
    plt.plot(points[:, 0], points[:, 1], 'o-')
    plt.plot(pnts[:, 0], pnts[:, 1], 'x-', lw=2)
    plt.show()

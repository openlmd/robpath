import time
import numpy as np

import calculate as calc


class Planning:
    def __init__(self):
        pass

    def get_range_values(self, v_min, v_max, v_dist):
        n_vals = np.round(((v_max - v_min) + v_dist) / v_dist)
        i_min = ((v_max + v_min) - (n_vals * v_dist)) / 2
        i_max = ((v_max + v_min) + (n_vals * v_dist)) / 2
        return np.arange(i_min, i_max + v_dist, v_dist)

    def get_grated(self, slice, dist):
        dist = dist - 1e-9
        fill_lines = []
        x_min = np.min([np.min(poly[:, 0]) for poly in slice])
        x_max = np.max([np.max(poly[:, 0]) for poly in slice])
        for x in self.get_range_values(x_min, x_max, dist):
            points = []
            for poly in slice:
                for k in range(len(poly)):
                    point = None
                    pnt1, pnt2 = poly[k-1], poly[k]
                    if pnt2[0] == x:
                        point = pnt2
                    else:
                        if (pnt1[0] < x and pnt2[0] > x) or (pnt1[0] > x and pnt2[0] < x):
                            m, b = calc.line2d(pnt1[:2], pnt2[:2])
                            point = np.array([x, m * x + b, pnt1[2]])
                    if point is not None:
                        points.append(point)
            if not points == []:
                points = np.array(points)
                indexes = np.argsort(points[:, 1])
                if len(indexes) % 2:
                    print 'ERROR!', len(indexes), 'tangent element finded'
                    indexes = indexes[:len(indexes)-1]
                #TODO: Group lines (pnt1, pnt2) with the same x value, in the same group of points.
                fill_lines.append(points[indexes])
        # Flips the array to start in the last point of the next line.
        lines = []
        pair = False
        for line in fill_lines:
            if pair:
                line = np.flipud(line)
            pair = not pair
            lines.append(line)
        return lines

    def get_path_from_fill_lines(self, fill_lines):
        orientation = np.array((0.0, 0.0, 0.0, 1.0))
        tool_path = []
        for line in fill_lines:
            for k in range(0, len(line), 2):
                pnt1, pnt2 = line[k], line[k+1]
                if len(tool_path):
                    last_point = tool_path[-1][0]
                    if np.all(last_point == pnt1):
                        tool_path[-1] = [pnt2, orientation, False]
                    else:
                        tool_path.append([pnt1, orientation, True])
                        tool_path.append([pnt2, orientation, False])
                else:
                    tool_path.append([pnt1, orientation, True])
                    tool_path.append([pnt2, orientation, False])
        return tool_path

    def get_path_from_slices(self, slices, track_distance=None, pair=False):
        t0 = time.time()
        orientation = np.array((0.0, 0.0, 0.0, 1.0))
        tool_path = []
        pair = False
        for k, slice in enumerate(slices):
            if slice is not None:
                if track_distance is None:
                    for contour in slice:
                        for point in contour[:-1]:
                            tool_path.append([point, orientation, True])
                        tool_path.append([contour[-1], orientation, False])
                else:
                    path = []
                    for contour in slice:
                        fill_lines = self.get_grated(slice, track_distance)
                        if pair:  # Controls the starting point of the next layer
                            fill_lines.reverse()
                        path.extend(self.get_path_from_fill_lines(fill_lines))
                    pair = not pair
                    tool_path.extend(path)
            t1 = time.time()
            print '[%.2f%%] Time to path %.3f s.' % ((100.0 * (k + 1)) / len(slices), t1 - t0)
        return tool_path


if __name__ == '__main__':
    import argparse
    from mesh import Mesh
    from mlabplot import MPlot3D

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--data', type=str,
                        default='../../data/piece7.stl',
                        help='path to input stl data file')
    args = parser.parse_args()
    filename = args.data

    # Triangle mesh is composed by a set of faces (triangles)
    mesh = Mesh(filename)

    t0 = time.time()
    slices = mesh.get_mesh_slices(0.5)
    t1 = time.time()
    print 'Time for slices:', t1 - t0

    planning = Planning()

    t0 = time.time()
    path = planning.get_path_from_slices(slices, 1.0)
    t1 = time.time()
    print 'Time for path:', t1 - t0

    laser_dist = 0
    travel_dist = 0
    for k, (point, orientation, process) in enumerate(path[:-1]):
        dist = calc.distance(path[k][0], path[k+1][0])
        if process:
            laser_dist = laser_dist + dist
        else:
            travel_dist = travel_dist + dist
    print 'Total laser distance:', laser_dist
    print 'Total travel distance:', travel_dist

    # # Get path with frames
    # _path = []
    # for position, orientation, process in path:
    #    frame, t = calc.quatpose_to_pose(position, orientation)
    #    _path.append([position, frame, process])

    mplot3d = MPlot3D()
    #mplot3d.draw_mesh(mesh)
    #mplot3d.draw_slices(slices)
    mplot3d.draw_path(path)
    mplot3d.show()
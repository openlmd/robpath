import re
import time
import struct
import numpy as np

import calculate as calc


def filter_polyline(points, dist=0.1, angl=0.01):
    if len(points) > 3:
        pnts = [points[0]]
        ang_min, ang_max = np.pi - angl, np.pi + angl
        for k in range(len(points)):
            point1, point2, point3 = pnts[-1], points[k-1], points[k]
            d12 = calc.distance2(point1, point2)
            #d23 = calc.distance2(point2, point3)
            if d12 <= dist:
                continue
            else:
                a123 = calc.angle(point1, point2, point3)
                if (a123 >= ang_min and a123 <= ang_max):
                    continue
            pnts.append(point2)
        pnts.append(points[-1])
        points = np.array(pnts)
    return points


class Mesh:
    def __init__(self, filename):
        # Bounding box
        self.origin = np.array([0.0, 0.0, 0.0])
        self.bpoint1 = np.array([0.0, 0.0, 0.0])
        self.bpoint2 = np.array([0.0, 0.0, 0.0])
        self.size = np.array([0.0, 0.0, 0.0])
        # Mesh loading routine
        self.valid = False
        self.triangles = []
        self.sink = 0
        if self.load_text_mesh(filename):
            self.valid = True
            self.bounding_box()
        elif self.load_binary_mesh(filename):
            self.valid = True
            self.bounding_box()
        if self.valid:
            self.translate(np.float32([0, 0, 0]))  # translates the piece to the origin
            #TODO: Resort only for calculation
            #self.resort_triangles()
        print '> Load file:', filename

    def load_binary_mesh(self, filename):
        try:
            with open(filename, 'rb') as f:
                data = f.read(84)  # skip header
                while data != '':
                    data = f.read(12)  # skip the normals entirely
                    if data != '':
                        tri = np.zeros((3, 3))
                        for j in range(3):
                            for i in range(3):
                                data = f.read(4)
                                tri[j, i] = struct.unpack('f', data)[0]
                    else:
                        break
                    data = f.read(2)  # skip the attribute bytes
                    self.triangles.append(tri)
            print "Loaded binary STL:", filename
        except:
            print "Unable to load binary STL"
            return False
        return True

    def load_text_mesh(self, filename):
        template = " *facet normal +(?P<xn>.+) (?P<yn>.+) (?P<zn>.+)\n"
        template += " *outer loop\n"
        template += " *vertex +(?P<x1>.+) (?P<y1>.+) (?P<z1>.+)\n"
        template += " *vertex +(?P<x2>.+) (?P<y2>.+) (?P<z2>.+)\n"
        template += " *vertex +(?P<x3>.+) (?P<y3>.+) (?P<z3>.+)\n"
        template += " *endloop\n"
        template += " *endfacet\n"
        try:
            f = open(filename)
            lines = f.read()
            f.close()
            pattern = re.compile(template, re.MULTILINE)
            results = pattern.findall(lines)
            if results:
                for xn, yn, zn, x1, y1, z1, x2, y2, z2, x3, y3, z3 in results:
                    tri = np.array([[float(x1), float(y1), float(z1)],
                                    [float(x2), float(y2), float(z2)],
                                    [float(x3), float(y3), float(z3)]])
                    self.triangles.append(tri)
            else:
                return False
            print "Loaded binary STL:", filename
        except:
            print "Unable to load text STL"
            return False
        return True

    def bounding_box(self):
        bx1, bx2 = 10000, -10000
        by1, by2 = 10000, -10000
        bz1, bz2 = 10000, -10000
        for tri in self.triangles:
            for k in range(3):
                if tri[k, 0] < bx1:
                    bx1 = tri[k, 0]
                if tri[k, 0] > bx2:
                    bx2 = tri[k, 0]
                if tri[k, 1] < by1:
                    by1 = tri[k, 1]
                if tri[k, 1] > by2:
                    by2 = tri[k, 1]
                if tri[k, 2] < bz1:
                    bz1 = tri[k, 2]
                if tri[k, 2] > bz2:
                    bz2 = tri[k, 2]
        self.bpoint1 = np.array([bx1, by1, bz1])
        self.bpoint2 = np.array([bx2, by2, bz2])
        self.x_min, self.x_max = bx1, bx2
        self.y_min, self.y_max = by1, by2
        self.z_min, self.z_max = bz1, bz2
        self.size = self.bpoint2 - self.bpoint1

    def translate(self, point):
        trans = point - self.bpoint1
        for k, tri in enumerate(self.triangles):
            self.triangles[k] = tri + trans
        self.bounding_box()

    def scale(self, scale):
        for k, tri in enumerate(self.triangles):
            self.triangles[k] = tri * scale
        self.bounding_box()

    def resort_triangles(self):
        """Sorts vertices from smaller to greater Z."""
        # Sorting the triangle according to height makes slicing then easier
        # point1, point2, point3 = resort(triangle)
        for k, tri in enumerate(self.triangles):
            self.triangles[k] = tri[tri[:, 2].argsort()]

    def get_range_values(self, v_min, v_max, v_dist):
        n_vals = np.round(((v_max - v_min) + v_dist) / v_dist)
        i_min = ((v_max + v_min) - (n_vals * v_dist)) / 2
        i_max = ((v_max + v_min) + (n_vals * v_dist)) / 2
        return np.arange(i_min, i_max + v_dist, v_dist)

    def get_zlevels(self, zdist):
        n_vals = np.round((self.z_max - self.z_min) / zdist)
        i_min = ((self.z_max + self.z_min) - (n_vals * zdist)) / 2
        i_max = ((self.z_max + self.z_min) + (n_vals * zdist)) / 2
        return np.arange(i_min, i_max + zdist, zdist) + 0.00001

    def get_z_intersect(self, triangle, z_level):
        """Gets the intersection line of the triangle with the plane in Z."""
        # Return the intersection of the tringle with the plane.
        # Returns None if the triangle does not intersect.
        point1, point2, point3 = triangle
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = point1, point2, point3
        intersect = None
        if z1 < z_level:
            dx21, dy21, dz21 = point2 - point1
            dx31, dy31, dz31 = point3 - point1
            dx32, dy32, dz32 = point3 - point2
            if z2 > z_level:
                xa = x1 + dx21 * (z_level - z1) / dz21
                ya = y1 + dy21 * (z_level - z1) / dz21
                if z3 > z_level:
                    xb = x1 + dx31 * (z_level - z1) / dz31
                    yb = y1 + dy31 * (z_level - z1) / dz31
                else:
                    xb = x2 + dx32 * (z_level - z2) / dz32
                    yb = y2 + dy32 * (z_level - z2) / dz32
                intersect = np.array([[xa, ya, z_level],
                                      [xb, yb, z_level]])
            elif z3 > z_level:
                xa = x1 + dx31 * (z_level - z1) / dz31
                ya = y1 + dy31 * (z_level - z1) / dz31
                xb = x2 + dx32 * (z_level - z2) / dz32
                yb = y2 + dy32 * (z_level - z2) / dz32
                intersect = np.array([[xa, ya, z_level],
                                      [xb, yb, z_level]])
        return intersect

    def get_slice(self, z_level):
        """Calculates the polygons in the slice for a plane."""
        unsorted_lines = []
        for triangle in self.triangles:
            if (triangle[0, 2] < z_level) and (triangle[2, 2] > z_level):
                intersection = self.get_z_intersect(triangle, z_level)
                unsorted_lines.append(intersection)
            elif (triangle[0, 2] == z_level) and (triangle[2, 2] == z_level):
                print "WARNING: Triangle in z_level!"
        if not unsorted_lines == []:
            # Arrange the line segments so that each segment leads to the
            # nearest available segment. This is accomplished by using two
            # list of lines, and at each step moving the nearest available
            # line segment from the unsorted pile to the next slot in the
            # sorted pile.
            epsilon = 1e-9
            polygons = []
            point1, point2 = unsorted_lines[0]
            polygon = [point1, point2]
            unsorted_lines.pop(0)
            while unsorted_lines:
                last_point = polygon[-1]
                do_flip, new_line = False, True
                for i, (point1, point2) in enumerate(unsorted_lines):
                    if calc.distance2(last_point, point1) < epsilon:
                        do_flip, new_line = False, False
                        break
                    if calc.distance2(last_point, point2) < epsilon:
                        do_flip, new_line = True, False
                        break
                    new_line = True
                point1, point2 = unsorted_lines[i]
                unsorted_lines.pop(i)
                if new_line:
                    polygons.append(np.array(polygon))
                    polygon = [point1, point2]
                else:
                    if do_flip:
                        if not (calc.distance2(polygon[-1], point1) < epsilon):
                            polygon.append(point1)
                    else:
                        if not (calc.distance2(polygon[-1], point2) < epsilon):
                            polygon.append(point2)
            polygons.append(np.array(polygon))
            return [filter_polyline(polygon, dist=0,
                                    angl=0) for polygon in polygons]  # Polygons filter
        else:
            return None

    def get_grated(self, slice, dist):
        fill_lines = []
        x_min = np.min([np.min(poly[:, 0]) for poly in slice])
        x_max = np.max([np.max(poly[:, 0]) for poly in slice])
        for x in self.get_range_values(x_min, x_max, dist):
            points = []
            for poly in slice:
                for k in range(len(poly)):
                    point = None
                    point1, point2 = poly[k-1], poly[k]
                    if point2[0] == x:
                        point = point2
                    else:
                        if (point1[0] < x and point2[0] > x) or (point1[0] > x and point2[0] < x):
                            m, b = calc.line2d(point1[:2], point2[:2])
                            point = np.array([x, m * x + b, point1[2]])
                    if point is not None:
                        points.append(point)
            if not points == []:
                points = np.array(points)
                indexes = np.argsort(points[:, 1])
                # ERROR!!!
                if len(indexes) % 2:
                    print 'ERROR IMPAR!', len(indexes)  #tangent element finded
                    indexes = indexes[:len(indexes)-1]
                #for k in range(0, len(indexes), 2):
                #    i1, i2 = indexes[k], indexes[k+1]
                #    fill_lines.append(np.array((points[i1,:], points[i2,:])))
                #TODO: Group lines (pnt1, pnt2) with the same x value, in the same group of points.
                fill_lines.append(points[indexes])
        return fill_lines

    def get_path_from_fill_lines(self, fill_lines):
        orientation = np.array((0.0, 0.0, 0.0, 1.0))
        pnt1, pnt2 = fill_lines[0][0], fill_lines[0][1]
        tool_path = []
        pair = False
        #offset = 10 # Adds an offset on the end of the track
        for line in fill_lines:
            # Flips the array to start in the last point of the next line.
            if pair:
                line = np.flipud(line)
            pair = not pair
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
            # Adds the offset path
            #last_point = tool_path[-1][0]
            #tool_path.append([last_point + np.float32([0, offset, 0]), orientation, False])
        return tool_path

    def get_path_from_slice(self, slice, track_distance, pair=False):
        path = []
        for contour in slice:
            fill_lines = self.get_grated(slice, track_distance)
            if pair:  # Controls the starting point of the next layer
                fill_lines.reverse()
            tool_path = self.get_path_from_fill_lines(fill_lines)
            path.extend(tool_path)
        return path

    def get_path_from_slices(self, slices):
        orientation = np.array((0.0, 0.0, 0.0, 1.0))
        tool_path = []
        for slice in slices:
            if slice is not None:
                for contour in slice:
                    for point in contour[:-1]:
                        tool_path.append([point, orientation, True])
                    tool_path.append([contour[-1], orientation, False])
        return tool_path

    def get_mesh_slices_path(self, layer_height, track_distance):
        t0 = time.time()
        slices, path = [], []
        levels = self.get_zlevels(layer_height)
        pair = False
        for k, z_level in enumerate(levels):
            slice = mesh.get_slice(z_level)
            if slice is not None:
                path.extend(self.get_path_from_slice(slice, track_distance, pair))
                pair = not pair
            slices.append(slice)
            t1 = time.time()
            print '[%.2f%%] Time to path %.3f s.' % ((100.0 * (k + 1)) / len(levels), t1 - t0)
        return slices, path


if __name__ == '__main__':
    import argparse
    from mlabplot import MPlot3D

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--data', type=str,
                        default='../../data/piece8.stl',
                        help='path to input stl data file')
    args = parser.parse_args()
    filename = args.data

    # Triangle mesh is composed by a set of faces (triangles)
    mesh = Mesh(filename)
    mesh.resort_triangles()

    # t0 = time.time()
    # slice = mesh.get_slice(0.1)
    # lines = mesh.get_grated(slice, 1.5)
    # t1 = time.time()
    # print 'Time for slice:', t1 - t0
    #
    # t0 = time.time()
    # path = mesh.get_path_from_slices([slice])
    # path = mesh.get_path_from_fill_lines(lines)
    # t1 = time.time()
    # print 'Time for path:', t1 - t0
    #
    # mplot3d = MPlot3D()
    # mplot3d.draw_slice(slice)
    # mplot3d.draw_path(path)
    # mplot3d.show()

    slices, path = mesh.get_mesh_slices_path(0.5, 2.0)
    path_contour = mesh.get_path_from_slices(slices)
    #print path_contour

    # TODO: Refactor to calculate Contour and Filler tool path.
    # TODO: Add options to interface (checkboxes).

    # Get path with frames
    #_path = []
    #for position, orientation, process in path:
    #    frame, t = calc.quatpose_to_pose(position, orientation)
    #    _path.append([position, frame, process])

    mplot3d = MPlot3D()
    mplot3d.draw_mesh(mesh)
    mplot3d.draw_slices(slices)
    #mplot3d.draw_path(path)
    mplot3d.draw_path(path_contour)
    #mplot3d.draw_path_tools(_path)
    mplot3d.show()

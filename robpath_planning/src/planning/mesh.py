import re
import time
import struct
import numpy as np

import calculate as calc
import polyline as poly


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
            return [poly.filter_polyline(polygon, dist=0.1) for polygon in polygons]  # Polygons filter
        else:
            return None

    def get_mesh_slices(self, layer_height):
        t0 = time.time()
        slices = []
        self.resort_triangles()
        levels = self.get_zlevels(layer_height)
        for k, z_level in enumerate(levels):
            slice = self.get_slice(z_level)
            slices.append(slice)
            t1 = time.time()
            print '[%.2f%%] Time to slice %.3f s.' % ((100.0 * (k + 1)) / len(levels), t1 - t0)
        return slices


if __name__ == '__main__':
    import argparse
    from mlabplot import MPlot3D

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--data', type=str,
                        default='../../data/piece7.stl',
                        help='path to input stl data file')
    args = parser.parse_args()
    filename = args.data

    # Triangle mesh is composed by a set of faces (triangles)
    mesh = Mesh(filename)
    mesh.resort_triangles()

    t0 = time.time()
    slice = mesh.get_slice(0.7)
    t1 = time.time()
    print 'Time for slice:', t1 - t0

    slices = mesh.get_mesh_slices(0.5)

    mplot3d = MPlot3D()
    mplot3d.draw_mesh(mesh)
    mplot3d.draw_slice(slice)
    mplot3d.draw_slices(slices)
    mplot3d.show()
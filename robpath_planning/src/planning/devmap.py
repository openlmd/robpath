import struct
import numpy as np
import logging
from scipy.spatial import Delaunay
from scipy.spatial import distance


class DevMap():
    def __init__(self):
        self.triangles_coords = []
        self.normals = []
        self.deviation_datas = []
        self.dev_vectors = []
        self.shift_triangles = []
        self.min_z_repair = 0

    def load_stl(self, filename):
        '''
        Load triangles and normals froma a binary *stl file
        '''
        with open(filename, 'rb') as f:
            # Skip header data
            data = f.read(84)
            while data != '':
                normal_strings = [f.read(4), f.read(4), f.read(4)]
                if normal_strings[2] != '':
                    normal = [struct.unpack('f', normal_strings[0])[0],
                              struct.unpack('f', normal_strings[1])[0],
                              struct.unpack('f', normal_strings[2])[0]]
                    self.normals.append(normal)
                    tri = np.zeros((3, 3))
                    for j in range(3):
                        for i in range(3):
                            data = f.read(4)
                            tri[j, i] = struct.unpack('f', data)[0]
                else:
                    break
                # skip the attribute bytes
                data = f.read(2)
                self.triangles_coords.append(tri)

    def load_deviation(self, filename):
        with open(filename, 'rb') as f:
            data = f.readlines()
            # split line into triangle number and its deviation
            for line in data:
                self.deviation_datas.append([int(line.split()[0]),
                                             float(line.split()[1])])

    def calculate_vectors(self):
        if len(self.normals) >= len(self.deviation_datas):
            for dev_pair in self.deviation_datas:
                vector = []
                for i in self.normals[dev_pair[0]]:
                    vector.append(i * dev_pair[1])
                self.dev_vectors.append(vector)
        else:
            print 'Insuficient normals data to calculate the deviation vectors'

    def calculate_shift_triangles(self):
        if len(self.dev_vectors) >= len(self.deviation_datas):
            for index, dev_pair in enumerate(self.deviation_datas):
                tri = self.triangles_coords[dev_pair[0]]
                # select only triangles from the up-face
                if self.normals[dev_pair[0]][2] > 0.4:
                    vect = self.dev_vectors[index]
                    shift_tri = []
                    for point in tri:
                        shift_point = []
                        for i_coord, coord in enumerate(point):
                            # TODO: Posible offset
                            shift_point.append(coord + vect[i_coord])
                        shift_tri.append(shift_point)
                    self.shift_triangles.append(shift_tri)
        else:
            print 'Insuficient deviation vectors to shif the triangles'

    def save_stl(self, filename, triangles=None):
        import stl
        if triangles is None:
            triangles = self.shift_triangles
        data = np.zeros(len(triangles), dtype=stl.mesh.Mesh.dtype)
        for n_tri, tri in enumerate(triangles):
            data['vectors'][n_tri] = np.array([tri[0],
                                               tri[1],
                                               tri[2]])
        surface = stl.mesh.Mesh(data)
        surface.save(filename)

    def get_min_z(self):
        for tri in self.shift_triangles:
            pass
        self.min_z_repair = min([min([point for point in tri],
                                 key=lambda x: x[2])[2] for tri in self.shift_triangles])


class Rays():
    def __init__(self):
        self.triangles = np.array([])
        self.tri_indices = []
        self.instersect_points = np.array([])
        self.tri_3d_indices = []
        self.tri_up_indices = []
        self.tri_down_indices = []
        self.ray = np.array([])
        self.ray_origin = np.array([])
        self.ray_vector = np.array([])
        self.ray_end = np.array([])
        self.epsilon = 0.0000001

    def load_triangles(self, triangles, delaunay=False):
        if delaunay:
            sup = np.concatenate(triangles)
            delaunay_tri = Delaunay(sup[:, 0:2])
            triangles = sup[delaunay_tri.simplices]
        self.triangles = np.array(triangles)
        self.instersect_points = np.array([])

    def load_ray(self, line_points):
        self.ray = np.array(line_points)
        self.ray_origin = self.ray[0]
        self.ray_end = self.ray[1]
        vect = self.ray[1] - self.ray[0]
        self.ray_vector = vect / np.linalg.norm(vect)
        self.instersect_points = np.array([])
        self.tri_3d_indices = []
        self.tri_up_indices = []
        self.tri_down_indices = []

    def invert_ray(self):
        self.load_ray(np.array([self.ray_end, self.ray_origin]))

    def point_to_plane(self, point, plane):
        '''Is a point placed in a plane surface Ax+By+Cz+D=0'''
        if len(plane) == 3:
            '3 points to construct a plane'
            vector0 = plane[1] - plane[0]
            vector1 = plane[2] - plane[0]
            normal = np.cross(vector0, vector1)
            normal = normal / np.linalg.norm(normal)
            A = normal[0]
            B = normal[1]
            C = normal[2]
            D = -(normal.dot(plane[0]))
        if len(plane) == 4:
            'ecuation of the plane'
            A = plane[0]
            B = plane[1]
            C = plane[2]
            D = plane[3]
        distance = normal.dot(point) + D
        if abs(distance) < 0.001:
            return 0
        return distance

    def triangle_intersects_ray(self, triangle, ray=None):
        '''
        Algoritmo de Moller-Trumbore por BLS
        Return 0: Not intersection, 1: On the same plane, 2:Backface inters,
               3: Frontface inters, 4: Intersects in line
        '''
        if type(ray) == list:
            ray = np.array(ray)
        if type(ray) == np.ndarray:
            self.load_ray(ray)
        if type(triangle) == list:
            triangle = np.array(triangle)
        edge1 = triangle[1] - triangle[0]
        edge2 = triangle[2] - triangle[0]
        h = np.cross(self.ray_vector, edge2)
        a = edge1.dot(h)
        logging.info('a: ' + str(a))
        if abs(a) < self.epsilon:
            logging.info('Triangulo e linea paralelos')
            if self.point_to_plane(self.ray_origin, triangle):
                return 0
            logging.info('Triangulo e linea no mesmo plano')
            return 1
        f = 1 / a
        s = self.ray_origin - triangle[0]
        u = f * np.dot(s, h)
        logging.info('u: ' + str(u))
        if (u < 0.0) or (u > 1.0):
            logging.info('u parameter outside bounds')
            return 0
        q = np.cross(s, edge1)
        v = f * np.dot(self.ray_vector, q)
        logging.info('v: ' + str(v))
        if (v < 0.0) or ((u + v) > 1.0):
            if v < 0.0:
                logging.info('v < 0: ' + str(v))
            if (u + v) > 1.0:
                logging.info('(u + v) > 1.0: ' + str(u + v))
            return 0
        # Pode ser un punto ou un segmento
        t = f * np.dot(edge2, q)
        logging.info('t: ' + str(t))
        if t > self.epsilon:
            intersec_point = self.ray_origin + self.ray_vector * t
            self.instersect_points = np.append(self.instersect_points,
                                               intersec_point)
            logging.info('Intersect 3D in point:')
            logging.info(intersec_point)
        else:
            logging.info('Intersect in line')
            return 4
        if a < 0:
            logging.info('Backfacing triangle')
            self.instersect_points = np.append(self.instersect_points, 2)
            return 2
        else:
            logging.info('Frontfacing triangle')
            self.instersect_points = np.append(self.instersect_points, 3)
            return 3

    def segment_intersects_ray(self, segment, ray=None):
        '''
        Checks if a projection over Z-plane of a ray intersecs a
        segment(def by 2 points)
        '''
        segment = np.array(segment)
        if type(ray) == list:
            ray = np.array(ray)
        if type(ray) == np.ndarray:
            self.load_ray(ray)
        seg = segment[:, :2]
        v1 = self.ray_origin[:2] - seg[0]
        v2 = seg[1] - seg[0]
        v3 = np.array([-self.ray_vector[1], self.ray_vector[0]])
        t1a = np.cross(v2, v1)
        t1b = np.dot(v2, v3)
        t1 = t1a / t1b
        t2 = np.dot(v1, v3) / np.dot(v2, v3)
        if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
            logging.info('Intersect 2D in point:')
            logging.info([self.ray_origin + t1 * self.ray_vector])
            return t1
        return False

    def triangle_intersects_ray_2d(self, triangle, ray=None):
        '''
        Cheks if the projections over Z-plane of a ray and a triangle
        are intersected
        '''
        triangle = np.array(triangle)
        if type(ray) == list:
            ray = np.array(ray)
        if type(ray) == np.ndarray:
            self.load_ray(ray)
        seg0 = np.array([triangle[0], triangle[1]])
        seg1 = np.array([triangle[1], triangle[2]])
        seg2 = np.array([triangle[2], triangle[0]])
        check0 = self.segment_intersects_ray(seg0)
        check1 = self.segment_intersects_ray(seg1)
        check2 = self.segment_intersects_ray(seg2)
        ints = []
        if check0:
            ints.append(check0)
        if check1:
            ints.append(check1)
        if check2:
            ints.append(check2)
        if len(ints) == 1:
            ints.append(ints[0])
        if len(ints) == 2:
            return np.array(sorted(ints))
        else:
            return False

    def select_triangles(self, segment=False):
        '''
        Select triangles in a 2D intersected by a ray or segment
        in a 2D projection
        '''
        indices_a = []
        indices_b = []
        # self.indices_dist = np.array([])
        # self.indices_dist = np.reshape(self.indices_dist, (3,))
        for index, tri in enumerate(self.triangles):
            distances = self.triangle_intersects_ray_2d(tri)
            if type(distances) == np.ndarray:
                indices_a.append(index)
                # self.indices_dist = np.stack((self.indices_dist, np.concatenate([np.array([index]), distances])))
        if not segment:
            self.tri_indices = indices_a
        else:
            self.invert_ray()
            for index, tri in enumerate(self.triangles):
                distances = self.triangle_intersects_ray_2d(tri)
                if type(distances) == np.ndarray:
                    indices_b.append(index)
            self.invert_ray()
            self.tri_indices = list(set(indices_a).intersection(indices_b))
        logging.info('selected in 2D projection :' + str(len(self.tri_indices)))

    def select_3d_triangles(self):
        '''Select triangles intersected by a ray in 3D space'''
        for index, tri in enumerate(self.triangles[self.tri_indices]):
            if self.triangle_intersects_ray(tri):
                self.tri_3d_indices.append(self.tri_indices[index])
            else:
                # 'Test if triangle is Up or Down the ray'
                if self.ray_vector[0]:
                    t = (tri[0][0] - self.ray_origin[0]) / self.ray_vector[0]
                else:
                    # 'Ray paralel to Y axis'
                    t = (tri[0][1] - self.ray_origin[1]) / self.ray_vector[1]
                z_ray = self.ray_origin[2] + self.ray_vector[2] * t
                if tri[0][2] > z_ray:
                    self.tri_up_indices.append(self.tri_indices[index])
                else:
                    self.tri_down_indices.append(self.tri_indices[index])
        self.instersect_points = self.instersect_points.reshape((len(self.instersect_points)/4,4))
        logging.info('selected in 3D space: ' + str(len(self.tri_3d_indices)))

    def order_points_distance(self, points=None, origin=None):
        '''Order points, from nearest to origin point'''
        if origin is None:
            origin = self.ray_origin
        if points is None:
            points = self.instersect_points[:, :3]
        distances = np.argsort(distance.cdist([origin], points))[0]
        return distances

    def get_segments(self):
        lines = np.array([])
        line = []
        dists = self.order_points_distance()
        for point_idx in dists:
            test_point = self.instersect_points[point_idx]
            if test_point[3] == 2:
                # Backfacing
                line.insert(0, test_point[:3])
            if test_point[3] == 3:
                # Frontfacing
                if len(line) == 0:
                    line.append(self.ray_origin)
                line.append(test_point[:3])
        if len(line) % 2 == 1:
            line.append(self.ray_end)
        lines = np.array(line)
        if len(lines) % 2 == 1:
            print 'Error getting segments'
        self.segments = lines

    def divide_segment(self, segment):
        self.load_ray(segment)
        # TODO: comprobar todo o que debe estar inicializado
        self.select_triangles(True)
        self.select_3d_triangles()
        self.get_segments()
        return self.segments


if __name__ == "__main__":
    d = DevMap()
    d.load_stl('P1ModelDevMap.stl')
    d.load_deviation('P1Deviations.cmr')
    d.calculate_vectors()
    d.calculate_shift_triangles()
    d.save_stl('surface.stl')

    r = Rays()
    tri = np.array([[131.25, 79.6875, 0.],
                    [137.5, 84.375, 0.],
                    [133.33332825, 78.125, 0.]])

    line = np.array([[133., 70., 1.],
                     [133., 80., 0]])

    line1 = np.array([[0., 69., 44.],
                      [67.5, 69., 44.]])
    line2 = np.array([[8.2, 69.8, 49.8],
                      [85.5, 69.7, 39.7]])
    r.triangle_intersects_ray(tri, line)
    r.segment_intersects_ray(line1, line2)

    r.load_triangles(d.shift_triangles)
    sup = np.concatenate(r.triangles)
    delaunay_tri = Delaunay(sup[:, 0:2])
    r.load_triangles(sup[delaunay_tri.simplices])
    r.load_ray(line1)
    r.select_triangles(False)
    r.select_3d_triangles()

    d.save_stl('surface_delaunay.stl', sup[delaunay_tri.simplices])
    d.save_stl('line_surface.stl', r.triangles[r.tri_indices])
    d.save_stl('cutted_triangles.stl', r.triangles[r.tri_3d_indices])
    d.save_stl('upside_triangles.stl', r.triangles[r.tri_up_indices])
    d.save_stl('downside_triangles.stl', r.triangles[r.tri_down_indices])

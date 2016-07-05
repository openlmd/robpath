import os
os.environ['ETS_TOOLKIT'] = 'qt4'

import numpy as np
from mayavi import mlab

import calculate as calc

BLACK = (0, 0, 0)
WHITE = (1, 1, 1)
RED = (1, 0, 0)
GREEN = (0, 1, 0)
BLUE = (0, 0, 1)


class MPlot3D():
    def __init__(self, mlab=None, scale=1.0):
        self.scale = scale
        if mlab is None:
            self._new_figure()

    def _new_figure(self):
        mlab.figure(1, size=(800, 600), bgcolor=BLACK, fgcolor=WHITE)

    #TODO: Add function to draw volume of work

    def draw_arrow(self, arrow, color=RED, scale=10):
        pnt, vec = arrow
        # scale = self.scale * scale
        mlab.quiver3d(pnt[0], pnt[1], pnt[2], vec[0], vec[1], vec[2],
                      color=color, mode='arrow', scale_factor=self.scale)

    def draw_frame(self, pose, scale=10, label=''):
        R, t = pose
        scale = self.scale * scale
        clr = [RED, GREEN, BLUE]
        vecs = R[:, 0], R[:, 1], R[:, 2]
        for k in range(3):
            mlab.quiver3d(t[0], t[1], t[2], vecs[k][0], vecs[k][1], vecs[k][2],
                          color=clr[k], mode='arrow', scale_factor=scale)
        mlab.text3d(t[0], t[1], t[2], label, scale=scale/5)

    def draw_frames(self, points, frames, scale=10):
        scale = self.scale * scale
        clr = [RED, GREEN, BLUE]
        vectors = [[], [], []]
        for frame in frames:
            vectors[0].append(frame[:, 0])
            vectors[1].append(frame[:, 1])
            vectors[2].append(frame[:, 2])
        vectors[0] = np.array(vectors[0])
        vectors[1] = np.array(vectors[1])
        vectors[2] = np.array(vectors[2])
        for k in range(3):
            mlab.quiver3d(points[:, 0], points[:, 1], points[:, 2],
                          vectors[k][:, 0], vectors[k][:, 1], vectors[k][:, 2],
                          color=clr[k], mode='arrow', scale_factor=scale)

    def draw_transformation(self, matrix1, matrix2, label1='', label2=''):
        self.draw_frame(calc.matrix_to_pose(matrix1), label=label1)
        self.draw_frame(calc.matrix_to_pose(matrix2), label=label2)
        self.draw_line(calc.matrix_to_pose(matrix1)[1],
                       calc.matrix_to_pose(matrix2)[1])

    def draw_plane(self, plane, points3d, color=(1, 0.1, 0)):
        # a plane is a*x+b*y+c*z+d=0
        # [a,b,c] is the normal. Thus, we have to calculate d and we're set
        a, b, c, d = plane
        #xmin, xmax = int(np.min(points3d[:,0])), int(np.max(points3d[:,0]))
        #ymin, ymax = int(np.min(points3d[:,1])), int(np.max(points3d[:,1]))
        ymin, ymax = -0.1, 0.1
        zmin, zmax = 0, 1.5 * np.max(points3d[:, 2])
        #x, z = np.mgrid[xmin:xmax:10, 0:500:10]
        #y = (a * x + c * z + d) / -b
        y, z = np.mgrid[ymin:ymax:0.09, zmin:zmax:0.09]
        x = (b * y + c * z + d) / -a
        print x, y, z
        mlab.mesh(x, y, z, color=color, opacity=0.5, transparent=True)

    def draw_line(self, point0, point1, color=WHITE, scale=0.25):
        scale = self.scale * scale
        mlab.plot3d([point0[0], point1[0]], [point0[1], point1[1]],
                    [point0[2], point1[2]], color=color, tube_radius=scale)

    def draw_points(self, points3d, color=WHITE, scale=1):
        scale = self.scale * scale
        mlab.points3d(points3d[:, 0], points3d[:, 1], points3d[:, 2],
                      color=color, scale_factor=scale)

    def draw_cloud(self, points3d, scale=1):
        scale = self.scale * scale
        mlab.points3d(points3d[:, 0], points3d[:, 1],
                      points3d[:, 2], points3d[:, 2],
                      colormap='jet', opacity=0.75, scale_factor=scale)
        mlab.outline()
        mlab.colorbar(title='Planar disparity (mm)')
        mlab.axes()

    def draw_lines(self, points3d, color=WHITE, scale=0.1):
        scale = self.scale * scale
        mlab.plot3d(points3d[:, 0], points3d[:, 1], points3d[:, 2],
                    color=color, tube_radius=scale)

    def draw_camera(self, camera_pose, color=(0.8, 0.8, 1), scale=0.5):
        camera_points = np.float32([[20, 16, 20],
                                    [-20, 16, 20],
                                    [-20, -16, 20],
                                    [20, -16, 20],
                                    [20, 16, 20],
                                    [0, 0, 0],
                                    [-20, 16, 20],
                                    [-20, -16, 20],
                                    [0, 0, 0],
                                    [20, -16, 20]]) * self.scale * scale
        camera_points = np.float32([np.dot(camera_pose[0], point) + camera_pose[1] for point in camera_points])
        self.draw_lines(camera_points, color=color, scale=scale)
        self.draw_frame(camera_pose, label='camera')

    def draw_layers(self, layers, scale=1.5):
        for layer in layers.itervalues():
            points = np.vstack([layer, layer[0]])
            self.draw_lines(points, color=WHITE)
            self.draw_points(points, color=(0.7, 0.1, 0.3), scale=scale)

    def draw_slice(self, slice, scale=0.3):
        if slice is not None:
            for points in slice:
                self.draw_lines(points, color=(0.8, 0.8, 0.8), scale=0.3*scale)
                self.draw_points(points, color=(0.6, 0.6, 0.6), scale=scale)

    def draw_slices(self, slices):
        for slice in slices:
            if slice is not None:
                self.draw_slice(slice)

    def draw_tools(self, points, frames, scale=10):
        scale = self.scale * scale
        vectors = []
        for frame in frames:
            vectors.append(frame[:, 2])
        vectors = np.array(vectors)
        points = points - scale * vectors
        mlab.quiver3d(points[:, 0], points[:, 1], points[:, 2],
                      vectors[:, 0], vectors[:, 1], vectors[:, 2],
                      color=(0, 0, 1), mode='arrow', scale_factor=scale)

    def draw_path_frames(self, path):
        points, frames = [], []
        for position, frame, process in path:
            points.append(position)
            frames.append(frame)
        points = np.array(points)
        frames = np.array(frames)
        self.draw_frames(points, frames)

    def draw_path_tools(self, path):
        points, frames = [], []
        for position, frame, process in path:
            points.append(position)
            frames.append(frame)
        points = np.array(points)
        frames = np.array(frames)
        self.draw_tools(points, frames)

    def _get_triangular_mesh(self, mesh):
        x, y, z = [], [], []
        for tri in mesh.triangles:
            for pnt in tri:
                x.append(pnt[0])
                y.append(pnt[1])
                z.append(pnt[2])
        triangles = [(i, i+1, i+2) for i in range(0, len(x), 3)]
        return x, y, z, triangles

    def draw_mesh(self, mesh, color=WHITE, opacity=0.7):
        x, y, z, triangles = self._get_triangular_mesh(mesh)
        mlab.triangular_mesh(x, y, z, triangles, color=color,
                             opacity=opacity, representation='surface')
        #mlab.triangular_mesh(x, y, z, triangles, colormap='bone',
        #                     opacity=0.8, representation='surface')
        mlab.triangular_mesh(x, y, z, triangles, color=(0, 0, 0),
                             line_width=1.0, representation='wireframe')
        #mlab.outline()
        #mlab.axes()

    def draw_path(self, path):
        points, vectors, processes = [], [], []
        for k in range(len(path)-1):
            points.append(path[k][0])
            vectors.append(path[k+1][0] - path[k][0])
            processes.append(path[k][2])
        points, vectors = np.array(points), np.array(vectors)
        processes = np.array(processes)
        pnts, vctrs = points[processes], vectors[processes]
        mlab.quiver3d(pnts[:, 0], pnts[:, 1], pnts[:, 2],
                      vctrs[:, 0], vctrs[:, 1], vctrs[:, 2],
                      color=(0.7, 0.5, 0.3), mode='2ddash',
                      scale_factor=1, line_width=5.0)
        pnts = points[np.bitwise_not(processes)]
        vctrs = vectors[np.bitwise_not(processes)]
        mlab.quiver3d(pnts[:, 0], pnts[:, 1], pnts[:, 2],
                      vctrs[:, 0], vctrs[:, 1], vctrs[:, 2],
                      color=(0.2, 0.4, 0.6), mode='2ddash',
                      scale_factor=1, line_width=2.0)

    def draw_point_cloud(self, points3d):
        mlab.clf()
        mlab.points3d(points3d[:, 0], points3d[:, 1], points3d[:, 2],
                      color=(1., 0.25, 0.25), opacity=0.75, scale_factor=0.001)
        mlab.outline()
        mlab.axes()

    def draw_working_area(self, width, height):
        color = (0.9, 0.9, 0.7)
        x, y = np.mgrid[0:height+1:10, 0:width+1:10]
        z = np.zeros(x.shape)
        mlab.surf(x, y, z, color=color, line_width=1.0,
                  representation='wireframe', warp_scale=self.scale)

    def outline(self):
        mlab.outline()

    def clear(self):
        mlab.clf()
        self._new_figure()

    def show(self):
        mlab.show()


if __name__ == '__main__':
    import doctest
    doctest.testmod()

    frame = np.float32([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
    point = np.float32([0, 0, 0])

    import calculate as calc

    arrow = np.float32([[0, 0, 0], [100, 0, 0]])

    pose_arrow = calc.rpypose_to_pose((0, 0, 100), (0, np.deg2rad(90), 0))
    trans_arrow = calc.matrix_invert(calc.pose_to_matrix(pose_arrow))
    print 'arrow quat:', calc.matrix_to_quatpose(trans_arrow)
    points = calc.transform_points(pose_arrow, arrow)
    arrow2 = (points[0], points[1]-points[0])
    print pose_arrow, points

    position = ((0, 20, 0), (np.deg2rad(-45), np.deg2rad(45), np.deg2rad(0)))
    pose = calc.rpypose_to_pose(*position)
    print 'quat:', calc.rpypose_to_quatpose(*position)
    trans = calc.matrix_compose([calc.pose_to_matrix(pose), calc.pose_to_matrix(pose_arrow)])
    print 'final pose:', calc.matrix_to_quatpose(trans)
    points = calc.transform_points(calc.matrix_to_pose(trans), arrow)
    arrow3 = (points[0], points[1]-points[0])
    print pose, points

    mplot3d = MPlot3D()
    mplot3d.draw_working_area(100, 100)
    mplot3d.draw_frame((frame, point), label='frame')
    mplot3d.draw_arrow(arrow)
    #mplot3d.draw_arrow(arrow2)
    mplot3d.draw_frame(calc.matrix_to_pose(trans_arrow))
    mplot3d.draw_arrow(arrow3)
    #mplot3d.draw_frame(calc.matrix_to_pose(calc.matrix_invert(trans)))
    mplot3d.show()

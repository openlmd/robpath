#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import planning.transformations as tf


class ShapeMarker():
    def __init__(self, frame_id='/world'):
        self.marker = Marker()
        self.marker.lifetime = rospy.Duration()

        self.set_frame(frame_id)

        self.set_scale()
        self.set_color()
        self.set_position()
        self.set_orientation()

    def set_frame(self, frame_id):
        self.marker.header.frame_id = frame_id

    def set_color(self, color=(1, 1, 1, 1)):
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = color[3]

    def set_scale(self, scale=(1, 1, 1)):
        self.marker.scale.x = scale[0]
        self.marker.scale.y = scale[1]
        self.marker.scale.z = scale[2]

    def set_position(self, position=(0, 0, 0)):
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]

    def set_orientation(self, orientation=(0, 0, 0, 1)):
        self.marker.pose.orientation.x = orientation[0]
        self.marker.pose.orientation.y = orientation[1]
        self.marker.pose.orientation.z = orientation[2]
        self.marker.pose.orientation.w = orientation[3]


class ArrowMarker(ShapeMarker):
    def __init__(self, length):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.ARROW
        self.set_length(length)
        self.matrix = np.eye(4)
        mat_inicio = tf.translation_matrix([0, 0, length])
        quat = [0, np.sin(np.deg2rad(45)),
                0, np.cos(np.deg2rad(45))]
        matrix_pos_0 = tf.quaternion_matrix(quat)
        self.mat_trans = np.dot(mat_inicio, matrix_pos_0)

    def set_length(self, length):
        self.set_scale((length, 0.1 * length, 0.1 * length))

    def set_pose(self, position, orientation):
        self.matrix = tf.quaternion_matrix(orientation)
        self.matrix[:3, 3] = position
        self.matrix = np.dot(self.matrix, self.mat_trans)
        orientation = tf.quaternion_from_matrix(self.matrix)
        position = self.matrix[:3, 3]
        self.set_orientation(orientation)
        self.set_position(position)


class CubeMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.CUBE


class SphereMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.SPHERE


class CylinderMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.CYLINDER


class LinesMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.LINE_STRIP
        self.marker.points = [Point(0, 0, 0)]
        self.set_size()

    def set_size(self, size=0.001):
        self.set_scale(scale=(size, size, 1.0))

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class SegmentsMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.LINE_LIST
        self.marker.points = [Point(0, 0, 0)]
        self.set_size()

    def set_size(self, size=0.001):
        self.set_scale(scale=(size, size, 1.0))

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class PointsMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.POINTS
        self.marker.points = [Point(0, 0, 0),
                              Point(1, 0, 0),
                              Point(1, 1, 0)]
        self.set_size()

    def set_size(self, size=0.025):
        self.set_scale(scale=(size, size, 1.0))

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class TextMarker(ShapeMarker):
    def __init__(self, text):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.TEXT_VIEW_FACING
        self.set_text(text)
        self.set_size()

    def set_text(self, text):
        self.marker.text = text

    def set_size(self, size=0.1):
        self.set_scale(scale=(1.0, 1.0, size))


class MeshMarker(ShapeMarker):
    def __init__(self, mesh_resource="package://proper_part/meshes/robot.dae", frame_id='/world'):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.mesh_resource = mesh_resource
        self.marker.mesh_use_embedded_materials = True


class TriangleListMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.TRIANGLE_LIST
        self.marker.points = [Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0),
                              Point(0, 0, 0), Point(1, 0, 0), Point(1, 0, 1)]
        #self.set_size()

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class ScanMarkers():
    def __init__(self):
        self.marker_array = MarkerArray()

        self.plane = CubeMarker()
        self.plane.set_frame('/workobject')
        self.plane.set_color((1.0, 0.0, 1.0, 0.7))
        self.marker_array.markers.append(self.plane.marker)
        self.path = LinesMarker()
        self.path.set_frame('/workobject')
        self.path.set_color((0.8, 0.0, 0.2, 1.0))
        self.marker_array.markers.append(self.path.marker)
        self.laser = SegmentsMarker()
        self.laser.set_frame('/workobject')
        self.laser.set_color((0.8, 0.0, 0.0, 1.0))
        self.marker_array.markers.append(self.laser.marker)
        for id, m in enumerate(self.marker_array.markers):
            m.id = id
        # marker_array.markers.append(CylinderMarker().marker)
        # marker_array.markers.append(PointsMarker().marker)
        self.offset = np.array((0, 0, 0))

    def set_plane_size(self, size):
        size = np.array(size) * 0.001
        self.offset = size / 2
        self.plane.set_scale(scale=size)

    def set_plane_position(self, position):
        position = np.array(position) * 0.001 + self.offset
        self.plane.set_position(position=position)

    def set_path(self, path):
        points = np.array([pose[0] for pose in path])
        self.path.set_points(0.001 * points)
        points = []
        for k in range(len(path) - 1):
            if len(path[k]) == 3:
                if path[k][2]:
                    points.append(path[k][0])
                    points.append(path[k+1][0])
        points = np.array(points)
        self.laser.set_points(0.001 * points)


class PartMarkers():
    def __init__(self):
        self.marker_array = MarkerArray()

        #self.marker = MeshMarker(mesh_resource="file://"+filename, frame_id="/workobject")
        self.mesh = TriangleListMarker()
        self.mesh.set_frame('/workobject')
        self.mesh.set_color((0.0, 0.5, 1.0, 0.7))
        self.marker_array.markers.append(self.mesh.marker)
        self.path = LinesMarker()
        self.path.set_frame('/workobject')
        self.path.set_color((0.7, 0.7, 0.7, 1.0))
        self.marker_array.markers.append(self.path.marker)
        self.laser = SegmentsMarker()
        self.laser.set_frame('/workobject')
        self.laser.set_color((0.8, 0.0, 0.0, 1.0))
        self.marker_array.markers.append(self.laser.marker)
        for id, m in enumerate(self.marker_array.markers):
            m.id = id

    def set_mesh(self, mesh):
        self.mesh.set_points(0.001 * np.vstack(mesh.triangles))

    def set_path(self, path):
        points = np.array([pose[0] for pose in path])
        self.path.set_points(0.001 * points)
        points = []
        for k in range(len(path) - 1):
            if len(path[k]) == 3:
                if path[k][2]:
                    points.append(path[k][0])
                    points.append(path[k+1][0])
        points = np.array(points)
        self.laser.set_points(0.001 * points)


class PathMarkers():
    def __init__(self):
        self.marker_array = MarkerArray()

        self.offset_position = 100
        self.quat = [0, np.sin(np.deg2rad(45)), 0, np.cos(np.deg2rad(45))]
        self.quat_inv = [0, -np.sin(np.deg2rad(45)), 0, np.cos(np.deg2rad(45))]

        self.path = LinesMarker()
        self.path.set_frame('/workobject')
        self.path.set_color((0.8, 0.0, 0.2, 1.0))
        self.marker_array.markers.append(self.path.marker)
        self.laser = SegmentsMarker()
        self.laser.set_frame('/workobject')
        self.laser.set_color((0.8, 0.2, 0.0, 1.0))
        self.marker_array.markers.append(self.laser.marker)
        self.arrow = ArrowMarker(0.075)
        self.arrow.set_color((0, 0, 0, 0))
        self.arrow.set_frame('/workobject')
        # self.arrow.set_position((0.2, 0.2, 0.2))
        # self.arrow.set_orientation((0, 0, 0, 1))
        self.marker_array.markers.append(self.arrow.marker)

        for id, m in enumerate(self.marker_array.markers):
            m.id = id

    def set_path(self, path):
        points = np.array([pose[0] for pose in path])
        self.path.set_points(0.001 * points)
        points = []
        for k in range(len(path) - 1):
            if len(path[k]) == 3:
                if path[k][2]:
                    points.append(path[k][0])
                    points.append(path[k+1][0])
        points = np.array(points) * 0.001
        self.laser.set_points(points)

    def set_pose(self, pose):
        if pose is not None:
            position, orientation = pose
            self.arrow.set_pose(position, orientation)
            self.arrow.set_color((0.0, 0.0, 0.7, 1.0))
        else:
            self.arrow.set_color((0.0, 0.0, 0.0, 0.0))


if __name__ == '__main__':
    rospy.init_node('markers')

    pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    pub_marker_array = rospy.Publisher(
        'visualization_marker_array', MarkerArray, queue_size=10)

    marker_array = MarkerArray()

    mesh_marker = MeshMarker(mesh_resource="package://proper_part/meshes/test.dae")

    cube = CubeMarker()
    cube.set_scale((0.1, 0.1, 0.1))
    cube.set_color((1, 1, 1, 0.7))
    cube.set_frame('/workobject')
    marker_array.markers.append(cube.marker)
    arrow = ArrowMarker(0.1)
    arrow.set_color((0, 0, 1, 1))
    arrow.set_frame('/workobject')
    arrow.set_position((0.2, 0.2, 0.2))
    arrow.set_orientation((0, 0, 0, 1))
    marker_array.markers.append(arrow.marker)

    # Renumber the marker IDs
    for id, m in enumerate(marker_array.markers):
        m.id = id

    k = 0
    while not rospy.is_shutdown():
        pub_marker.publish(mesh_marker.marker)
        pub_marker_array.publish(marker_array)
        k = k + 0.01
        cube.set_position((k, 0, 0))
        rospy.sleep(1.0)

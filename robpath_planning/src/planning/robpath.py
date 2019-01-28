import os
import json
import numpy as np
import calculate as calc
import xml.etree.ElementTree as ET
from xml.dom import minidom

from mesh import Mesh
from planning import Planning


class Part(Mesh):
    def __init__(self, filename):
        Mesh.__init__(self, filename)
        # Set parameters
        self.set_process(8, 1000, 0, 20)
        self.set_track(0.7, 2.5, 0.45)
        self.set_powder(20, 15, 5)
        self.filling = 0.0
        self.one_dir_fill = True
        self.invert_fill_y = True
        self.invert_fill_x = True
        self.invert_control = True
        self.repair_work = False

    def resize_mesh(self, size):
        self.scale(size / self.size)

    def transform_mesh(self, mesh):
        return mesh

    def set_process(self, speed, power, focus, travel=20):
        self.speed = speed
        self.travel_speed = travel
        self.power = power
        self.focus = focus

    def get_process(self):
        return self.speed, self.power, self.focus

    def set_track(self, height, width, overlap):
        self.track_height = height
        self.track_width = width
        self.track_overlap = overlap
        self.track_distance = (1 - overlap) * width

    def get_track(self):
        return self.track_height, self.track_width, self.track_overlap

    def set_powder(self, carrier, stirrer, turntable):
        self.carrier = carrier
        self.stirrer = stirrer
        self.turntable = turntable

    def get_powder(self):
        return self.carrier, self.stirrer, self.turntable


class RobPath():
    def __init__(self):
        self.part = None
        self.name = None
        self.parts = []
        self.planning = Planning()
        self.base_frame = np.eye(4)
        self.origin = np.array([.0, .0, .0])

    def load_mesh(self, filename):
        self.part = Part(filename)
        self.part.name = str(len(self.parts)) + '_' + os.path.basename(filename)
        self.name = self.part.name
        self.parts.append(self.part)

    def load_xml(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        self.path = []
        lineas = []
        tool_path = []
        for line in root.iter('Desplazamiento'):
            if line.attrib['tipo'] == 'linea3D':
                puntos = []
                for punto in line.iter('Punto'):
                    parray = np.array([float(punto.find('x').text),
                                float(punto.find('y').text),
                                float(punto.find('z').text)])
                    puntos.append(parray)
                lineas.append(puntos)
            if line.attrib['tipo'] == 'BuildCutVolume3D':
                for cut_layers in line.iter('Paths'):
                    for cut_layer in cut_layers.iter('SubPath'):
                        for line_3ds in cut_layer.iter('Paths'):
                            for line_3d in line_3ds.iter('SubPath'):
                                puntos = []
                                for punto in line_3d.iter('Punto'):
                                    parray = np.array([float(punto.find('x').text),
                                                float(punto.find('y').text),
                                                float(punto.find('z').text)])
                                    puntos.append(parray)
                                lineas.append(puntos)

        tool_path = self.planning.get_path_from_fill_lines(lineas)
        focus = 0
        tool_path = self.planning.translate_path(tool_path, np.array([0, 0, focus]))
        self.path.extend(tool_path)

    def save_xml(self, filename, path):
        top = ET.Element('Programa')
        comment = ET.Comment('Generado por AIMEN')
        top.append(comment)
        tray = ET.SubElement(top, 'Trayectoria')
        despl = None
        punt = None
        orde = 1
        for k in range(len(path)):
            p, q, process = path[k]
            if process:
                orde = 1
                despl = ET.SubElement(tray, 'Desplazamiento')
                despl.set("tipo", "linea3D")
            else:
                orde = 2
            punt = ET.SubElement(despl, 'Punto')
            children = ET.XML('<root><Orden>%i</Orden><x>%f</x><y>%f</y><z>%f</z></root>' %(orde,p[0],p[1],p[2]))
            punt.extend(children)
        rough_string = ET.tostring(top, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        filename = 'robpath.xml'
        with open(filename, 'w') as f:
            f.writelines(reparsed.toprettyxml(indent="  "))

    def select_part(self, name):
        for part in self.parts:
            if part.name == name:
                self.part = part
                return name

    def translate(self, position):
        if self.name is None:
            trans = position - self.origin
            [part.translate(trans + part.origin) for part in self.parts]
            self.origin = position
        else:
            self.part.translate(position)

    def load_base_frame(self, filename='../../data/base_frame.json'):
        try:
            with open(filename) as data_file:
                frame_data = json.load(data_file)
            if frame_data['frame']['type'] == 'Matrix':
                t = frame_data['frame']['t']
                u = frame_data['frame']['u']
                v = frame_data['frame']['v']
                w = frame_data['frame']['w']
                self.base_frame = np.array((
                    (u[0], v[0], w[0], t[0]),
                    (u[1], v[1], w[1], t[1]),
                    (u[2], v[2], w[2], t[2]),
                    (0.0, 0.0, 0.0, 1.0)
                    ), dtype=np.float64)
            else:
                self.set_base_frame(frame_data['frame']['t'], frame_data['frame']['quat'])
        except IOError as error:
            print error
        except:
            print 'Unexpected load_base_frame error'

    def set_base_frame(self, position, orientation):
        self.base_frame = calc.quatpose_to_matrix(position, orientation)

    def transform_path(self, path, r=None):
        tpath = []
        for position, orientation, process in path:
            matrix = calc.quatpose_to_matrix(position, orientation)
            if r is None:
                tmatrix = np.dot(self.base_frame, matrix)
            else:
                base_matrix = calc.rpypose_to_matrix([0,0,0], [0,0,np.radians(r)])
                tmatrix = np.dot(base_matrix, matrix)
            trans, quat = calc.matrix_to_quatpose(tmatrix)
            tpath.append((trans, orientation, process))
        return tpath

    def transform_slice(self, slice, r):
        tslice = []
        tslices = []
        for s in slice:
            for position in s:
                matrix = calc.rpypose_to_matrix(position, [0,0,0])
                base_matrix = calc.rpypose_to_matrix([0,0,0], [0,0,np.radians(r)])
                tmatrix = np.dot(base_matrix, matrix)
                trans, quat = calc.matrix_to_quatpose(tmatrix)
                tslice.append([trans[0],trans[1],trans[2]])
            tslices.append(tslice)
        nslice = np.array(tslices)
        return nslice

    def init_process(self):
        self.k = 0
        self.path = []
        self.slices = []
        self.pair = False
        if self.name is None and len(self.parts) > 0:
            zmin, zmax = self.parts[0].position[2], (self.parts[0].position + self.parts[0].size)[2]
            for part in self.parts:
                if part.position[2] < zmin:
                    zmin = part.position[2]
                if (part.position + part.size)[2] > zmax:
                    zmax = (part.position + part.size)[2]
                if part.repair_work:
                    zmin = part.min_z_repair
            self.levels = self.parts[0].get_zlevels(self.parts[0].track_height, zmin=zmin, zmax=zmax)
        else:
            self.levels = self.part.get_zlevels(self.part.track_height)
        return self.levels

    def init_coating(self):
        from scipy.spatial import cKDTree
        self.path = []
        coating_start_point = np.array(self.part.position)
        coating_size = np.array(self.part.size)
        coating_direction = np.array([1.0, 0.0, 0.0])
        coating_distance = 5.0
        coating_start_point = coating_start_point + coating_direction * coating_distance
        triangles = self.part.devmap.shift_triangles # self.part.rays.triangles
        print 'INTIT COAT'
        print coating_start_point
        print coating_size
        track_points = np.array(self.get_coating_track(coating_start_point, coating_direction, triangles))
        print 'Track:'
        print len(track_points)
        print '----'
        tree = cKDTree(track_points)
        rows_to_fuse = tree.query_pairs(r=0.01, output_type='ndarray')
        print 'Puntos para fusionar: ' + str(len(rows_to_fuse))
        print rows_to_fuse
        print '----'
        ind = np.lexsort((track_points[:,1],track_points[:,0]))
        print track_points
        print '----'
        ordered_track = track_points[ind]
        print ordered_track
        print '----'
        max_distance = 20.0
        break_points = []
        min_distance = 0.1
        discarted_points = []
        for n in range(len(ordered_track)-1):
            dist = np.absolute(np.linalg.norm(ordered_track[n+1]-ordered_track[n]))
            if dist > max_distance:
                break_points.append(n)
#TODO: separar os cordons antes de descartar puntos cercanos
        tool_paths = []
        if len(break_points) == 0:
            tool_path = self.planning.get_path_from_coating_track(ordered_track)
            tool_paths.append(tool_path)
        else:
            print 'Break_points:'
            print len(break_points)
            print break_points
            for break_point in range(len(break_points)):
                if break_point == 0:
                    tool_path = self.planning.get_path_from_coating_track(ordered_track[:break_points[break_point]])
                    tool_paths.append(tool_path)
                elif break_points[break_point] - break_points[break_point-1] < 3:
                    pass
                else:
                    tool_path = self.planning.get_path_from_coating_track(ordered_track[break_points[break_point-1]+1:break_points[break_point]])
                    tool_paths.append(tool_path)
            tool_path = self.planning.get_path_from_coating_track(ordered_track[break_points[-1]+1:])
            tool_paths.append(tool_path)
#TODO: aqui filtrar os puntos cercanos
        for tool_path in tool_paths:
            self.path.extend(tool_path)

    def get_coating_track(self, plane_point, plane_normal, triangles):
        # TODO: Pasar a planning ou clase que corresponda
        track_points = []
        for triangle in triangles:
            sideness_A = np.dot(plane_normal, np.array(triangle[0]) - plane_point)
            sideness_B = np.dot(plane_normal, np.array(triangle[1]) - plane_point)
            sideness_C = np.dot(plane_normal, np.array(triangle[2]) - plane_point)
            if sideness_A * sideness_B < 0:
                # Punto de corte con segmento 0,1
                r = np.dot(plane_normal, plane_point - np.array(triangle[0])) / np.dot(plane_normal, np.array(triangle[1]) - np.array(triangle[0]))
                p = np.array(triangle[0]) + r * (np.array(triangle[1]) - np.array(triangle[0]))
                track_points.append(p)
            if sideness_A * sideness_C < 0:
                # Punto de corte con segmento 0,2
                r = np.dot(plane_normal, plane_point - np.array(triangle[0])) / np.dot(plane_normal, np.array(triangle[2]) - np.array(triangle[0]))
                p = np.array(triangle[0]) + r * (np.array(triangle[2]) - np.array(triangle[0]))
                track_points.append(p)
            if sideness_B * sideness_C < 0:
                # Punto de corte con segmento 1,2
                r = np.dot(plane_normal, plane_point - np.array(triangle[1])) / np.dot(plane_normal, np.array(triangle[2]) - np.array(triangle[1]))
                p = np.array(triangle[1]) + r * (np.array(triangle[2]) - np.array(triangle[1]))
                track_points.append(p)
        return track_points

    def update_process(self, filled=True, contour=False):
        tool_path = []
        slices = []
        degrees = []
        if self.part.invert_fill_y:
            self.part.invert_control = not self.part.invert_control
        if self.name is None:
            for part in self.parts:
                slices.append(part.get_slice(self.levels[self.k]))
                degrees.append(part.filling)
        else:
            slices.append(self.part.get_slice(self.levels[self.k]))
            degrees.append(self.part.filling)
        for n, slice in enumerate(slices):
            if slice is not None:
                self.slices.append(slice)
                if filled:
                    if degrees[n] == 0.0:
                        tool_path = self.planning.get_path_from_slices(
                            [slice], self.part.track_distance, self.pair, focus=self.part.focus,
                            one_dir=self.part.one_dir_fill, invert=self.part.invert_control)
                        self.path.extend(tool_path)
                    else:
                        tslice = self.transform_slice(slice, degrees[n])
                        tool_path = self.planning.get_path_from_slices(
                            [tslice], self.part.track_distance, self.pair, focus=self.part.focus,
                            one_dir=self.part.one_dir_fill, invert=self.part.invert_control)
                        tool_path = self.transform_path(tool_path, -degrees[n])
                        self.path.extend(tool_path)
                if contour:
                    tool_path = self.planning.get_path_from_slices(
                        [slice], focus=self.part.focus)
                    self.path.extend(tool_path)
        if self.part.invert_fill_x:
            self.pair = not self.pair
        self.k = self.k + 1
        print 'k, levels:', self.k, len(self.levels)
        return tool_path


    def update_process_alfa(self, filled=True, contour=False):
        tool_path = []
        slices = []
        degrees = []
        if self.part.invert_fill_y:
            self.part.invert_control = not self.part.invert_control
        # TODO: Fix contour when zlevel = 0
        if contour and (self.levels[self.k] < 0.1):
            self.levels[self.k] = 0.1
        if self.name is None:
            for part in self.parts:
                slices.append(part.get_slice(self.levels[self.k]))
                degrees.append(part.filling)
        else:
            slices.append(self.part.get_slice(self.levels[self.k]))
            degrees.append(self.part.filling)
        for n, slice in enumerate(slices):
            if slice is not None:
                self.slices.append(slice)
                if filled:
                    tool_path = self.planning.get_path_from_slices(
                        [slice], self.part.track_distance, self.pair, focus=self.part.focus,
                        one_dir=self.part.one_dir_fill, invert=self.part.invert_control, degrees=degrees[n])
                    if self.part.repair_work:
                        tool_path = self.repair_tool_path(tool_path)
                    self.path.extend(tool_path)
                if contour:
                    tool_path = self.planning.get_path_from_slices(
                        [slice], focus=self.part.focus)
                    #  TODO: Repair
                    self.path.extend(tool_path)
        if self.part.invert_fill_x:
            self.pair = not self.pair
        self.k = self.k + 1
        print 'k, levels:', self.k, len(self.levels)
        return tool_path

    def repair_tool_path(self, tool_path):
        new_tool_path = []
        segments = []
        segment = []
        for tool in tool_path:
            if tool[2]:
                segment.append(tool[0])
                if len(segment) == 2:
                    segments = self.part.rays.divide_segment(segment)
                    segment.delete[0]
            else:
                segment.append(tool[0])
                segments = self.part.rays.divide_segment(segment)
                segment = []
            if len(segments) > 0:
                if len(segments) % 2 == 1:
                    print 'Error: repair toolpath, segments not pair points'
                for index, point in enumerate(segments):
                    new_tool_path.append((point, tool[1], not(index % 2)))
                segments = []
        return new_tool_path

    def get_process_time(self):
        time = 0
        if len(self.path) > 0:
            length = self.planning.path_length(self.path)
            time = self.planning.path_time(
                length, self.part.speed, self.part.travel_speed)
        return length[0] / self.part.speed, length[1] / self.part.travel_speed


if __name__ == "__main__":
    import argparse
    from mlabplot import MPlot3D

    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--mesh', type=str,
                        default='../../data/piece0.stl',
                        help='path to input stl data file')
    args = parser.parse_args()

    filename = args.mesh

    robpath = RobPath()
    robpath.load_mesh(filename)
    robpath.part.set_track(0.5, 2.5, 0.4)
    robpath.part.set_process(8, 1000, 0.0)
    levels = robpath.init_process()
    for k, level in enumerate(levels):
        robpath.update_process(filled=True, contour=True)

    mplot3d = MPlot3D()
    #mplot3d.draw_mesh(robpath.part)
    #mplot3d.draw_slices(slices)
    mplot3d.draw_path(robpath.path)
    #mplot3d.draw_path_tools(_path)
    mplot3d.show()

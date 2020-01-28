import os
import json
import numpy as np
import calculate as calc
import planning as plan
import math
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
        self.perimeters = 0

    def get_track(self):
        return self.track_height, self.track_width, self.track_overlap

    def set_powder(self, carrier, stirrer, turntable):
        self.carrier = carrier
        self.stirrer = stirrer
        self.turntable = turntable

    def get_powder(self):
        return self.carrier, self.stirrer, self.turntable


class GCodePart():
    def __init__(self):
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
        self.perimeters = 0

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
        self.dynamic_params = []
        self.params_group = []
        self.lenght_tracks = None

    def load_mesh(self, filename):
        self.part = Part(filename)
        self.part.name = str(len(self.parts)) + '_' + os.path.basename(filename)
        self.name = self.part.name
        self.parts.append(self.part)

    def load_xml(self, filename):
        #TODO: modificar path
        tree = ET.parse(filename)
        root = tree.getroot()
        self.path = []
        self.dynamic_params = []
        self.params_group = []
        lineas = []
        tool_path = []
        parameters_index = -1
        if root.tag == 'Programa':
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
        elif root.tag == 'program':
            for part in root:
                if part.tag == 'part':
                    for layer in part:
                        if layer.tag == 'layer':
                            if layer.find('parameters') is not None:
                                parameters_index = len(self.dynamic_params)
                            else:
                                parameters_index = -1
                            for laserTrack in layer:
                                if laserTrack.tag == 'parameters':
                                    self.dynamic_params.append({'speed':laserTrack.attrib['speed'],
                                                                'laserPower':laserTrack.attrib['laserPower']})
                                if laserTrack.tag == 'laserTrack':
                                    puntos = []
                                    for point in laserTrack:
                                        if point.tag == 'point':
                                            parray = np.array([float(point.attrib['x']),
                                                    float(point.attrib['y']),
                                                    float(point.attrib['z'])])
                                            puntos.append(parray)
                                            self.params_group.append(parameters_index)
                                    lineas.append(puntos)
        tool_path = self.planning.get_path_from_fill_lines(lineas)
        # focus = 0
        # tool_path = self.planning.translate_path(tool_path, np.array([0, 0, focus]))
        self.path.extend(tool_path)
        '''print self.path:
        [(array([0.45, 0.45, 0.45]), array([-0.113,  0.   ,  0.   ,  0.994]), True), 
        (array([50.45,  0.45,  0.45]), array([-0.113,  0.   ,  0.   ,  0.994]), False), 
        (array([ 0.45, 50.45,  0.45]), array([-0.113,  0.   ,  0.   ,  0.994]), True), 
        (array([50.45, 50.45,  0.45]), array([-0.113,  0.   ,  0.   ,  0.994]), True), 
        (array([70.45, 60.45,  0.45]), array([-0.113,  0.   ,  0.   ,  0.994]), False), 
        (array([90.45, 90.45,  1.46]), array([-0.113,  0.   ,  0.   ,  0.994]), True), 
        (array([ 0.45, 90.45,  1.46]), array([-0.113,  0.   ,  0.   ,  0.994]), False)]
        '''

    def load_gcode(self, filename):
        #TODO: Cargar E para FDM
        import pygcode
        from pygcode import Line
        self.part = GCodePart()
        z_offset = 0
        z = 0.0 + z_offset
        x = 0.0
        y = 0.0
        self.path = []
        extrusions = []
        e_value = 0.0
        puntos = []
        lineas = []
        tool_path = []
        with open(filename, 'r') as fh:
            for line_text in fh.readlines():
                line = Line(line_text)
                if len(line.block.words) == 2:
                    continue
                extrude_move = False
                if len(line.block.modal_params) > 0:
                    if line.block.modal_params[0].letter == 'E':
                        #TODO: Pode dar erro se hai mais de un model_param
                        extrude_move = True
                for block in line.block.gcodes:
                    if type(block) == pygcode.gcodes.GCodeLinearMove and extrude_move:
                        if block.Z is not None:
                            z = block.Z + z_offset
                            print 'OLLO: Proceso en Z'
                        if block.X is not None and block.Y is not None:
                            e_value += line.block.modal_params[0].value
                            x = block.X
                            y = block.Y
                            parray = np.array([x, y, z])
                            if z >= 0.0:
                                puntos.append(parray)
                    elif type(block) == pygcode.gcodes.GCodeRapidMove or type(block) == pygcode.gcodes.GCodeLinearMove:
                        if block.X is not None:
                            x = block.X
                        if block.Y is not None:
                            y = block.Y
                        if block.Z is not None:
                            z = block.Z + z_offset
                        if puntos:
                            if len(puntos) > 1:
                                # REVIEW:  se hai varios puntos sen proceso, vaise o ultimo
                                lineas.append(puntos)
                                extrusions.append(e_value)
                            puntos = []
                            e_value = 0
                        parray = np.array([x, y, z])
                        if z >= 0.0:
                            puntos.append(parray)
        if len(puntos) > 1:
            lineas.append(puntos)
            extrusions.append(e_value)
        #TODO: Crear funcion especifica para anhadir orientacions aos ptos
        tool_path = self.planning.get_path_from_fill_lines(lineas)
        # focus = 0
        # tool_path = self.planning.translate_path(tool_path, np.array([0, 0, focus]))
        self.path.extend(tool_path)
        return [extrusions]

    def load_orientations(self, filename):
        '''
        Load text file containing new orientation for path points
        [[[[[148.713, 100.0, 1.7, 0, 1.065402062407932],...]]]]]
        '''
        import ast
        self.path = []
        layer_ordered_points = []
        with open(filename, 'r') as fh:
            for line_text in fh.readlines():
                layer_ordered_points = ast.literal_eval(line_text)
        self.path.extend(layer_ordered_points)
        """ points = []
        for l in layer_ordered_points:
            for t in l:
                for p in t:
                    ## TODO: REvisar porque hai un nivel mais
                    for incognita in p:
                        points.append(incognita)
        self.update_orientations(points) """

    def update_orientations(self, points):
        '''
        Update path points with loaded orientations
        '''
        yaw = 0
        new_points = []
        for point in points:
            point_ori_rpy = np.radians(np.append(np.array(point[3:5]), yaw))
            point_xyz, point_ori = calc.rpypose_to_quatpose(np.array(point[:3]), point_ori_rpy)
            new_points.append([point_xyz, point_ori])
        if len(new_points) == len(self.path):
            for i, point in enumerate(new_points):
                n = 0
                if not np.array_equal(point[0], self.path[i][0]):
                    n += 1
                    self.path[i][0] = point[0]
                print 'Modified n points: ', n
                self.path[i][1] = point[1]
        else:
            print 'WARNING: Path and orientation poits mismatch'
            print len(new_points)
            print len(self.path)

    def save_xml_to_file(self, filename, top):
        rough_string = ET.tostring(top, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        with open(filename, 'w') as f:
            f.writelines(reparsed.toprettyxml(indent="  "))

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
        self.save_xml_to_file(filename.split('.')[0] + '_old.xml', top)
        self.save_xml_new(filename, path)

    def save_xml_new(self, filename, path):
        top = ET.Element('program')
        comment = ET.Comment('Generado por AIMEN')
        top.append(comment)
        part = ET.SubElement(top, 'part')
        h = -10.0
        layer = None
        track = None
        for k in range(len(path)):
            p, q, process = path[k]
            if h < p[2]:
                h = p[2]
                layer = ET.SubElement(part, 'layer')
                track = ET.SubElement(layer, 'laserTrack')
            point = ET.SubElement(track, 'point')
            point.set('x', str(p[0]))
            point.set('y', str(p[1]))
            point.set('z', str(p[2]))
            if not process:
                track = ET.SubElement(layer, 'laserTrack')
        self.save_xml_to_file(filename, top)

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
        for layer in path:
            tlayer = []
            for track in layer:
                ttrack = []
                for position, orientation in track:
                    matrix = calc.quatpose_to_matrix(position, orientation)
                    if r is None:
                        tmatrix = np.dot(self.base_frame, matrix)
                    else:
                        base_matrix = calc.rpypose_to_matrix([0,0,0], [0,0,np.radians(r)])
                        tmatrix = np.dot(base_matrix, matrix)
                    trans, quat = calc.matrix_to_quatpose(tmatrix)
                    ttrack.append([trans, orientation])
                tlayer.append(ttrack)
            tpath.append(tlayer)
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
        #TODO:
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
                # self.slices.append(slice)
                offset_values = []
                offset_values.append([-self.part.track_distance * off for off in range(self.part.perimeters + 1)])
                offsets_slices, infill_slices = plan.multi_offset(slice, offset_values[0], self.levels[self.k])
                if filled:
                    tool_path = self.planning.get_path_from_slices(
                        [infill_slices[:]], self.part.track_distance, self.pair, focus=self.part.focus,
                        one_dir=self.part.one_dir_fill, invert=self.part.invert_control, degrees=degrees[n])
                    if self.part.repair_work:
                        tool_path = self.repair_tool_path(tool_path)
                    self.path.extend(tool_path)
                if contour:
                    tool_path = self.planning.get_path_from_slices(
                        [offsets_slices[:]], focus=self.part.focus)
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

    def calculate_extrusions(self, extrusions = None, lengths=None, height=None, width=None):
        if not lengths:
            lengths=self.lenght_tracks
        process_speed = self.part.speed
        filament_diameter = 2.85
        feedrate = 0.0
        if extrusions is None:
            if not height:
                height=self.part.track_height
            if not width:
                width=self.part.track_width
            filament_volume = (width-height)*height+3.141593*(height/2)**2
            filament_extrudded = 3.141593*(filament_diameter/2)**2
            unit_extrusion = filament_volume / filament_extrudded
            feedrate = unit_extrusion * process_speed * 60
            extrusions = []
            for layer in lengths:
                extrusion_layer = []
                for track in layer:
                    extrusion_layer.append(track*unit_extrusion)
                extrusions.append(extrusion_layer)
        else:
            feedrate = extrusions[0][0] / (lengths[0][0] / (60 * process_speed))
        return extrusions, feedrate

    def get_process_time(self):
        time = 0
        if len(self.path) > 0:
            length = self.planning.path_length(self.path)
            time = self.planning.path_time(
                length[:2], self.part.speed, self.part.travel_speed)
        #TODO: extrusions con length[2]
        self.lenght_tracks = length[2]
        return length[0] / self.part.speed, length[1] / self.part.travel_speed


if __name__ == "__main__":
    import argparse
    from mlabplot import MPlot3D

    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--mesh', type=str,
                        default='/home/baltasar/catkin_ws/src/robpath/robpath_planning/data/piece0.stl',
                        help='path to input stl data file')
    args = parser.parse_args()

    filename = args.mesh

    robpath = RobPath()
    robpath.load_mesh(filename)
    robpath.part.set_track(1.5, 2.5, 0.4)
    robpath.part.set_process(8, 1000, 0.0)
    robpath.part.filling = 0.0
    robpath.part.invert_fill_x = False
    robpath.part.invert_fill_y = True
    robpath.planning.start_point_dir = 'max'
    levels = robpath.init_process()
    #robpath.levels = [12.5]
    for k, level in enumerate(robpath.levels):
        robpath.update_process_alfa(filled=True, contour=True)

    mplot3d = MPlot3D()
    #mplot3d.draw_mesh(robpath.part)
    #mplot3d.draw_slices(slices)
    mplot3d.draw_path(robpath.path)
    #mplot3d.draw_path_tools(_path)
    mplot3d.show()

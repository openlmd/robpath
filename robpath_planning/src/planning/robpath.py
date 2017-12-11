import os
import json
import numpy as np
import calculate as calc
import xml.etree.ElementTree as ET

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
            self.levels = self.parts[0].get_zlevels(self.parts[0].track_height, zmin=zmin, zmax=zmax)
        else:
            self.levels = self.part.get_zlevels(self.part.track_height)
        return self.levels

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


    def get_process_time(self):
        time = 0
        if len(self.path) > 0:
            length = self.planning.path_length(self.path)
            time = self.planning.path_time(
                length, self.part.speed, self.part.travel_speed)
        return time


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

import os
import json
import numpy as np
import calculate as calc

from mesh import Mesh
from planning import Planning


class Part(Mesh):
    def __init__(self, filename):
        Mesh.__init__(self, filename)
        # Set parameters
        self.set_process(8, 1000, 0)
        self.set_track(0.7, 2.5, 0.45)
        self.set_powder(20, 15, 5)

    def resize_mesh(self, size):
        self.scale(size / self.size)

    def transform_mesh(self, mesh):
        return mesh

    def set_process(self, speed, power, focus):
        self.speed = speed
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

    def transform_path(self, path):
        tpath = []
        for position, orientation, process in path:
            matrix = calc.quatpose_to_matrix(position, orientation)
            tmatrix = np.dot(self.base_frame, matrix)
            trans, quat = calc.matrix_to_quatpose(tmatrix)
            tpath.append((trans, quat, process))
        return tpath

    def init_process(self):
        self.k = 0
        self.path = []
        self.slices = []
        self.pair = False
        self.levels = self.part.get_zlevels(self.part.track_height)
        return self.levels

    def update_process(self, filled=True, contour=False):
        tool_path = []
        slice = self.part.get_slice(self.levels[self.k])
        if slice is not None:
            self.slices.append(slice)
            if filled:
                tool_path = self.planning.get_path_from_slices(
                    [slice], self.part.track_distance, self.pair, focus=self.part.focus)
                self.pair = not self.pair
                self.path.extend(tool_path)
            if contour:
                tool_path = self.planning.get_path_from_slices(
                    [slice], focus=self.part.focus)
                self.path.extend(tool_path)
        self.k = self.k + 1
        print 'k, levels:', self.k, len(self.levels)
        return tool_path


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

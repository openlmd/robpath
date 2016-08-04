from mesh import Mesh
from planning import Planning
from rapid import Rapid


class RobPath():
    def __init__(self):
        self.mesh = None
        self.rob_parser = Rapid()
        self.planning = Planning()

    def load_mesh(self, filename):
        self.mesh = Mesh(filename)

    def translate_mesh(self, position):
        self.mesh.translate(position)

    def resize_mesh(self, size):
        self.mesh.scale(size / self.mesh.size)

    def set_track(self, height, width, overlap):
        self.track_height = height
        self.track_width = width
        self.track_overlap = overlap
        self.track_distance = (1 - overlap) * width

    def set_process(self, speed, power, focus):
        self.rob_parser.track_speed = speed
        self.rob_parser.power = power
        self.track_focus = focus

    def set_powder(self, carrier_gas, stirrer, turntable):
        self.rob_parser.carrier_gas = carrier_gas
        self.rob_parser.stirrer = stirrer
        self.rob_parser.turntable = turntable

    def init_process(self):
        self.k = 0
        self.path = []
        self.slices = []
        self.pair = False
        self.levels = self.mesh.get_zlevels(self.track_height)
        return self.levels

    def update_process(self, filled=True, contour=False):
        tool_path = []
        slice = self.mesh.get_slice(self.levels[self.k])
        if slice is not None:
            self.slices.append(slice)
            if filled:
                tool_path = self.planning.get_path_from_slices(
                    [slice], self.track_distance, self.pair, focus=self.track_focus)
                self.pair = not self.pair
                self.path.extend(tool_path)
            if contour:
                tool_path = self.planning.get_path_from_slices(
                    [slice], focus=self.track_focus)
                self.path.extend(tool_path)
        self.k = self.k + 1
        print 'k, levels:', self.k, len(self.levels)
        return tool_path

    def save_rapid(self, filename='etna.mod', directory='ETNA'):
        routine = self.rob_parser.path2rapid(self.path)
        self.rob_parser.save_file(filename, routine)
        self.rob_parser.upload_file(filename, directory)
        print routine


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
    robpath.set_track(0.5, 2.5, 0.4)
    robpath.set_focus(0.0)
    levels = robpath.init_process()
    for k, level in enumerate(levels):
        robpath.update_process(filled=True, contour=True)

    mplot3d = MPlot3D()
    #mplot3d.draw_mesh(robpath.mesh)
    #mplot3d.draw_slices(slices)
    mplot3d.draw_path(robpath.path)
    #mplot3d.draw_path_tools(_path)
    mplot3d.show()

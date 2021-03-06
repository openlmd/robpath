#!/usr/bin/env python
import os
import numpy as np
import json
import datetime
os.environ['QT_API'] = 'pyqt'
os.environ['ETS_TOOLKIT'] = 'qt4'

# To be able to use PySide or PyQt4 and not run in conflicts with traits,
# we need to import QtGui and QtCore from pyface.qt
from pyface.qt import QtGui, QtCore

# from mayavi import mlab
from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item
from mayavi.core.ui.api import MlabSceneModel, SceneEditor
from tvtk.pyface.api import Scene

#from PyQt4 import QtGui, QtCore, uic
from PyQt4 import uic

from planning.rapid import Rapid
from planning.robpath import RobPath
from planning.mlabplot import MPlot3D


class Visualization(HasTraits):
    scene = Instance(MlabSceneModel, ())

    @on_trait_change('scene.activated')
    def update_plot(self):
        # This function is called when the view is opened.
        self.scene.mlab.view(0, 65, 150)
        self.scene.background = (0.2, 0.2, 0.2)

    # the layout of the dialog screated
    view = View(Item('scene', editor=SceneEditor(scene_class=Scene),
                     height=600, width=800, show_label=False), resizable=True)


class QMayavi(QtGui.QWidget):
    def __init__(self, parent=None):
        super(QMayavi, self).__init__(parent)

        self.setWindowTitle("Mesh Viewer")
        layout = QtGui.QGridLayout(self)
        layout.setSpacing(3)

        ui_mlab = Visualization().edit_traits(parent=self, kind='subpanel').control
        layout.addWidget(ui_mlab, 0, 0)
        self.mlab = MPlot3D(mlab=True)

        self.progress = QtGui.QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setTextVisible(False)
        self.progress.setMinimumHeight(6)
        self.progress.setMaximumHeight(6)
        layout.addWidget(self.progress, 1, 0)

    def drawWorkingArea(self, width=300, height=200):
        self.mlab.clear()
        self.mlab.draw_working_area(width, height)
        #self.mlab.draw_points(np.float32([[0, 0, 0], [height, width, 100]]),
        #                      scale=0.1)  # working area
        #self.mlab.outline()

    def drawMesh(self, mesh):
        self.drawWorkingArea()
        self.mlab.draw_mesh(mesh)

    def drawMeshes(self, meshes, selected=None):
        self.drawWorkingArea()
        for mesh in meshes:
            if selected is None or selected == mesh.name:
                self.mlab.draw_mesh(mesh, color=(1, 0, 0))
            else:
                self.mlab.draw_mesh(mesh, color=mesh.color)

    def drawPath(self, path, color=(1, 0, 0)):
        self.mlab.draw_path(path, color)

    def drawSlice(self, slices, path):
        self.mlab.draw_slice(slices[-1])
        #self.plot.mlab.draw_path(tool_path)
        self.mlab.draw_path(path)


class RobPathUI(QtGui.QMainWindow):
    def __init__(self):
        super(RobPathUI, self).__init__()
        path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        uic.loadUi(os.path.join(path, 'resources', 'robpath.ui'), self)

        fileNam = os.path.realpath(__file__)
        fileDir = os.path.dirname(fileNam)
        fileDir = fileDir + "/configs/config.json"

        with open(fileDir) as data_file:
            self.settings = json.load(data_file)

        self.plot = QMayavi()
        self.boxPlot.addWidget(self.plot)
        self.plot.drawWorkingArea()

        self.btnLoadMesh.clicked.connect(self.btnLoadMeshClicked)
        self.btnSelectAll.clicked.connect(self.btnSelectAllClicked)
        self.btnSelectMesh.activated.connect(self.btnSelectMeshClicked)
        self.btnProcessMesh.clicked.connect(self.btnProcessMeshClicked)
        self.btnSaveRapid.clicked.connect(self.btnSaveRapidClicked)

        self.sbPositionX.valueChanged.connect(self.changePosition)
        self.sbPositionY.valueChanged.connect(self.changePosition)
        self.sbPositionZ.valueChanged.connect(self.changePosition)

        self.sbSizeX.valueChanged.connect(self.changeSize)
        self.sbSizeY.valueChanged.connect(self.changeSize)
        self.sbSizeZ.valueChanged.connect(self.changeSize)
        self.sbFilling.valueChanged.connect(self.changeFilling)
        self.checkBoxSliceInvertY.stateChanged.connect(self.changeFilling)
        self.checkBoxSliceInvertX.stateChanged.connect(self.changeFilling)
        self.checkBoxSliceOnedir.stateChanged.connect(self.changeFilling)

        self.actionLoadSettings.triggered.connect(self.loadSettings)
        self.actionSaveSettings.triggered.connect(self.saveSettings)

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        self.dirname = path
        self.selected = None
        self.processing = False
        self.timer = QtCore.QTimer(self.plot)
        self.timer.timeout.connect(self.updateProcessing)
        self.new_xml = False
        self.manual_stop = False
        self.stop_layer = None

        self.robpath = RobPath()
        self.rapid = Rapid()
        self.enableParts(False)

    def enableParts(self, value):
        self.btnProcessMesh.setEnabled(value)
        self.btnSelectAll.setEnabled(value)
        self.sbPositionX.setEnabled(value)
        self.sbPositionY.setEnabled(value)
        self.sbPositionZ.setEnabled(value)
        self.sbSizeX.setEnabled(value)
        self.sbSizeY.setEnabled(value)
        self.sbSizeZ.setEnabled(value)
        self.sbFilling.setEnabled(value)
        self.checkBoxSliceInvertY.setEnabled(value)
        self.checkBoxSliceInvertX.setEnabled(value)
        self.checkBoxSliceOnedir.setEnabled(value)

    def blockSignals(self, value):
        self.sbPositionX.blockSignals(value)
        self.sbPositionY.blockSignals(value)
        self.sbPositionZ.blockSignals(value)
        self.sbSizeX.blockSignals(value)
        self.sbSizeY.blockSignals(value)
        self.sbSizeZ.blockSignals(value)
        self.sbFilling.blockSignals(value)
        self.checkBoxSliceInvertY.blockSignals(value)
        self.checkBoxSliceInvertX.blockSignals(value)
        self.checkBoxSliceOnedir.blockSignals(value)

    def loadSettings(self):
        filename = QtGui.QFileDialog.getOpenFileName(
            None, 'Open file', None,
            'Configuration (*json)')
        if filename.split('.')[-1] == 'json':
            with open(filename) as data_file:
                self.settings = json.load(data_file)

        if "overlap" in self.settings["configuration"]:
             self.sbOverlap.setValue(self.settings["configuration"]["overlap"])
        if "width" in self.settings["configuration"]:
            self.sbWidth.setValue(self.settings["configuration"]["width"])
        if "height" in self.settings["configuration"]:
            self.sbHeight.setValue(self.settings["configuration"]["height"])
        if "process_speed" in self.settings["configuration"]:
            self.sbSpeed.setValue(self.settings["configuration"]["process_speed"])
        if "travel_speed" in self.settings["configuration"]:
            self.sbTravel.setValue(self.settings["configuration"]["travel_speed"])
        if "stop_layer" in self.settings["configuration"]:
            self.stop_layer = self.settings["configuration"]["stop_layer"]
        if "disparity" in self.settings["configuration"]:
            self.disparity_work = self.settings["configuration"]["disparity"]
        if "laser_type" in self.settings["configuration"]:
            if not self.settings["configuration"]["laser_type"] in self.settings["limits"]["laser_type"]:
                QtGui.QMessageBox.warning(self, "Cannot configure laser",
                        "The selected laser type is not implemented.",
                        QtGui.QMessageBox.Ok, QtGui.QMessageBox.NoButton,
                        QtGui.QMessageBox.NoButton)
            else:
                self.rapid.laser_type = self.settings["configuration"]["laser_type"]
        if "feeder_type" in self.settings["configuration"]:
            if not self.settings["configuration"]["feeder_type"] in self.settings["limits"]["feeder_type"]:
                QtGui.QMessageBox.warning(self, "Cannot configure feeder",
                        "The selected feeder type is not implemented.",
                        QtGui.QMessageBox.Ok, QtGui.QMessageBox.NoButton,
                        QtGui.QMessageBox.NoButton)
            else:
                self.rapid.feeder_type = self.settings["configuration"]["feeder_type"]
        if "offsets" in self.settings["configuration"]:
            if "line" in self.settings["configuration"]["offsets"]:
                self.rapid.offset = self.settings["configuration"]["offsets"]["line"]
            if "z_dir" in self.settings["configuration"]["offsets"]:
                self.rapid.offset_z = self.settings["configuration"]["offsets"]["z_dir"]
        if "start_point" in self.settings["configuration"]:
            self.robpath.planning.start_point = self.settings["configuration"]["start_point"]
        if "start_point_dir" in self.settings["configuration"]:
            self.robpath.planning.start_point_dir = self.settings["configuration"]["start_point_dir"]
        if "fill_direction" in self.settings["configuration"]:
            self.sbFilling.setValue(self.settings["configuration"]["fill_direction"])
        if "one_way" in self.settings["configuration"]:
            self.checkBoxSliceOnedir.setChecked(self.settings["configuration"]["one_way"])
        if "reverse_y_start" in self.settings["configuration"]:
            self.checkBoxSliceInvertY.setChecked(self.settings["configuration"]["reverse_y_start"])
        if "reverse_x_start" in self.settings["configuration"]:
            self.checkBoxSliceInvertX.setChecked(self.settings["configuration"]["reverse_x_start"])
        if "tool" in self.settings["configuration"]:
            self.rapid.tool = self.settings["configuration"]["tool"]
        if "base_frame" in self.settings["configuration"]:
            self.rapid.workobject = self.settings["configuration"]["base_frame"]

    def saveSettings(self):
        self.settings["configuration"]["overlap"] = self.sbOverlap.value()
        self.settings["configuration"]["width"] = self.sbWidth.value()
        self.settings["configuration"]["height"] = self.sbHeight.value()
        self.settings["configuration"]["process_speed"] = self.sbSpeed.value()
        self.settings["configuration"]["travel_speed"] = self.sbTravel.value()
        self.settings["configuration"]["stop_layer"] = self.stop_layer
        self.settings["configuration"]["laser_type"] = self.rapid.laser_type
        self.settings["configuration"]["offsets"]["line"] = self.rapid.offset
        self.settings["configuration"]["offsets"]["z_dir"] = self.rapid.offset_z
        self.settings["configuration"]["start_point"] = self.robpath.planning.start_point
        self.settings["configuration"]["start_point_dir"] = self.robpath.planning.start_point_dir
        self.settings["configuration"]["fill_direction"] = self.sbFilling.value()
        self.settings["configuration"]["one_way"] = self.checkBoxSliceOnedir.isChecked()
        self.settings["configuration"]["reverse_y_start"] = self.checkBoxSliceInvertY.isChecked()
        self.settings["configuration"]["reverse_x_start"] = self.checkBoxSliceInvertX.isChecked()
        self.settings["configuration"]["tool"] = self.rapid.tool
        self.settings["configuration"]["base_frame"] = self.rapid.workobject

        filename = QtGui.QFileDialog.getSaveFileName(
            None, 'Save file', None,
            'All Files (*);;Settings Files (*.json)')
        if len(filename)==0:
            print 'Type a file name'
            return
        if filename.split('.')[-1] != 'json':
            filename = filename + '.json'

        with open(filename, 'w') as data_file:
            data_file.write(json.dumps(self.settings, sort_keys=True,
                                       indent=2, separators=(',', ': ')))

    def changePosition(self):
        x = self.sbPositionX.value()
        y = self.sbPositionY.value()
        z = self.sbPositionZ.value()
        self.robpath.translate(np.float32([x, y, z]))
        # self.plot.drawMesh(self.robpath.part)
        self.plot.drawMeshes(self.robpath.parts, selected=self.robpath.name)

    def changeSize(self):
        sx = self.sbSizeX.value() + 0.001
        sy = self.sbSizeY.value() + 0.001
        sz = self.sbSizeZ.value() + 0.001
        self.robpath.part.resize_mesh(np.float32([sx, sy, sz]))
        self.changePosition()

    def changeTrack(self):
        height = self.sbHeight.value() + 0.00001
        width = self.sbWidth.value() + 0.00001
        overlap = 0.01 * self.sbOverlap.value()
        self.robpath.part.set_track(height, width, overlap)

    def changeProcess(self):
        speed = self.sbSpeed.value()
        power = self.sbPower.value()
        focus = self.sbFocus.value()
        travel_speed = self.sbTravel.value()
        self.robpath.part.set_process(speed, power, focus, travel_speed)
        self.rapid.set_process(speed, power, travel_speed)

    def changePowder(self):
        carrier = self.sbCarrier.value()
        stirrer = self.sbStirrer.value()
        turntable = self.sbTurntable.value()
        self.robpath.part.set_powder(carrier, stirrer, turntable)
        self.rapid.set_powder(carrier, stirrer, turntable)

    def changeFilling(self):
        if self.robpath.part:
            self.robpath.part.filling = self.sbFilling.value()
            self.robpath.part.one_dir_fill = self.checkBoxSliceOnedir.isChecked()
            self.robpath.part.invert_fill_y = self.checkBoxSliceInvertY.isChecked()
            self.robpath.part.invert_fill_x = self.checkBoxSliceInvertX.isChecked()

    def updatePosition(self, position):
        x, y, z = position
        self.sbPositionX.setValue(x)
        self.sbPositionY.setValue(y)
        self.sbPositionZ.setValue(z)

    def updateSize(self, size):
        sx, sy, sz = size
        self.sbSizeX.setValue(sx)
        self.sbSizeY.setValue(sy)
        self.sbSizeZ.setValue(sz)

    def updateTrack(self, (height, width, overlap)):
        self.sbHeight.setValue(height)
        self.sbWidth.setValue(width)
        self.sbOverlap.setValue(overlap)

    def updateProcess(self, (speed, power, focus)):
        self.sbSpeed.setValue(speed)
        self.sbPower.setValue(power)
        self.sbFocus.setValue(focus)

    def updatePowder(self, (carrier, stirrer, turntable)):
        self.sbCarrier.setValue(carrier)
        self.sbStirrer.setValue(stirrer)
        self.sbTurntable.setValue(turntable)

    def updateFilling(self, part):
        self.sbFilling.setValue(part.filling)
        self.checkBoxSliceOnedir.setChecked(part.one_dir_fill)
        self.checkBoxSliceInvertY.setChecked(part.invert_fill_y)
        self.checkBoxSliceInvertX.setChecked(part.invert_fill_x)

    def btnLoadMeshClicked(self):
        try:
            filename = QtGui.QFileDialog.getOpenFileName(
                self.plot, 'Open file', self.dirname,
                'Mesh Files (*.stl);; Process file (*xml);; Deviation map (*cmr);; Gcode (*gcode)')
            if filename:
                if filename.split('.')[-1] == 'stl':
                    self.enableParts(True)
                    self.dirname = os.path.dirname(filename)
                    self.robpath.load_mesh(filename)
                    self.setWindowTitle('Mesh Viewer: %s' % filename)
                    self.btnSelectMesh.addItems([self.robpath.name])
                    self.btnSelectMesh.setCurrentIndex(self.btnSelectMesh.count())
                    self.btnProcessMesh.setEnabled(True)
                    self.robpath.part.filling = self.sbFilling.value()
                    self.robpath.part.one_dir_fill = self.checkBoxSliceOnedir.isChecked()
                    self.robpath.part.invert_fill_y = self.checkBoxSliceInvertY.isChecked()
                    self.robpath.part.invert_fill_x = self.checkBoxSliceInvertX.isChecked()
                    self.changeProcess()
                    self.changeTrack()
                    self.changePowder()
                    self.updateMeshData(self.robpath.name)
                elif filename.split('.')[-1] == 'cmr':
                    # TODO: Check if part exists
                    self.robpath.part.load_deviation(filename)
                    self.robpath.part.repair_work = True
                    self.robpath.part.save_stl('surface_robpath.stl')
                    self.robpath.part.devmap.save_stl('surface_robpath_delaunay.stl', self.robpath.part.rays.triangles)
                elif filename.split('.')[-1] == 'xml':
                    self.robpath.load_xml(filename)
                    self.new_xml = True
                    self.timer.start(100)
                    self.btnSaveRapid.setEnabled(True)
                    length = self.robpath.planning.path_length(self.robpath.path)
                    laser_time = length[0] / self.sbSpeed.value()
                    travel_time = length[1] / self.sbTravel.value()
                    time = laser_time + travel_time
                    time_str = (str(round(time / 60, 2)) + ' min:\n'
                                + str(round(laser_time / 60, 2)) + ' process + '
                                + str(round(travel_time / 60, 2)) + ' travel')
                    print time_str
                    self.labelTime.setText(time_str)
                elif filename.split('.')[-1] == 'gcode':
                    self.robpath.load_gcode(filename)
                    self.new_xml = True
                    self.timer.start(100)
                    self.btnSaveRapid.setEnabled(True)
                    length = self.robpath.planning.path_length(self.robpath.path)
                    laser_time = length[0] / self.sbSpeed.value()
                    travel_time = length[1] / self.sbTravel.value()
                    time = laser_time + travel_time
                    time_str = (str(round(time / 60, 2)) + ' min:\n'
                                + str(round(laser_time / 60, 2)) + ' process + '
                                + str(round(travel_time / 60, 2)) + ' travel')
                    print time_str
                    self.labelTime.setText(time_str)

        except AttributeError as error:
            print error
        except ValueError as error:
            print error
        #self.plot.drawMesh(self.robpath.part)
        self.plot.drawMeshes(self.robpath.parts, self.robpath.name)

    def updateMeshData(self, name):
        self.robpath.select_part(name)
        self.blockSignals(True)
        self.updatePosition(self.robpath.part.origin)
        self.updateSize(self.robpath.part.size)
        self.updateTrack(self.robpath.part.get_track())
        self.updateProcess(self.robpath.part.get_process())
        self.updatePowder(self.robpath.part.get_powder())
        self.updateFilling(self.robpath.part)
        self.blockSignals(False)

    def btnSelectAllClicked(self):
        self.robpath.name = None
        self.updatePosition(self.robpath.origin)
        self.plot.drawMeshes(self.robpath.parts)

    def btnSelectMeshClicked(self):
        self.robpath.name = self.btnSelectMesh.currentText()
        self.updateMeshData(self.robpath.name)
        self.plot.drawMeshes(self.robpath.parts, selected=self.robpath.name)

    def updateProcessing(self):
        if self.new_xml:
            self.new_xml = False
            self.plot.drawPath(self.robpath.path, color=(1.0, 1.0, 0))
            self.timer.stop()
            return
        try:
            if self.robpath.k < len(self.robpath.levels):
                self.robpath.update_process_alfa(filled=self.chbFilled.isChecked(),
                                            contour=self.chbContour.isChecked())
                #self.plot.drawSlice(self.robpath.slices, self.robpath.path)
                # self.plot.drawPath(self.robpath.path, self.robpath.part.color)
                self.plot.progress.setValue(100.0 * self.robpath.k / len(self.robpath.levels))
                self.btnSaveRapid.setEnabled(True)
                laser_time, travel_time = self.robpath.get_process_time()
                time = laser_time + travel_time
                time_str = (str(round(time / 60, 2)) + ' min:\n'
                            + str(round(laser_time / 60, 2)) + ' process + '
                            + str(round(travel_time / 60, 2)) + ' travel')
                self.labelTime.setText(time_str)
                n_levels = str(len(self.robpath.levels)) + ' layers'
                self.labelLevels.setText(n_levels)
            else:
                self.processing = False
                self.timer.stop()
                self.btnSaveRapid.setEnabled(True)
                laser_time, travel_time = self.robpath.get_process_time()
                time = laser_time + travel_time
                time_str = (str(round(time / 60, 2)) + ' min:\n'
                            + str(round(laser_time / 60, 2)) + ' process + '
                            + str(round(travel_time / 60, 2)) + ' travel')
                self.labelTime.setText(time_str)
                n_levels = str(len(self.robpath.levels)) + ' layers'
                self.labelLevels.setText(n_levels)
                self.plot.drawPath(self.robpath.path, self.robpath.part.color)
            if self.stop_layer == self.robpath.k:
                self.plot.drawPath(self.robpath.path, self.robpath.part.color)
                self.timer.stop()
                self.manual_stop = True
        except IndexError as error:
            print error

    def btnProcessMeshClicked(self):
        if self.manual_stop:
            self.manual_stop = False
            self.timer.start(100)
            return
        if len(self.robpath.parts) == 0:
            msg = QtGui.QMessageBox()
            msg.setIcon(QtGui.QMessageBox.Warning)
            msg.setText("Error")
            msg.setInformativeText("A mesh must be loaded")
            msg.setWindowTitle("Robpath")
            #msg.setDetailedText("The details are as follows:")
            msg.setStandardButtons(QtGui.QMessageBox.Ok)
            retval = msg.exec_()
            return
        if self.robpath.part.repair_work and self.disparity_work == "coating":
            self.btnCoatFaceClicked()
            return
        if self.processing:
            self.timer.stop()
            self.processing = False
        else:
            self.plot.drawWorkingArea()
            self.changeTrack()
            self.changeProcess()
            self.changePowder()
            self.robpath.init_process()
            self.processing = True
            self.timer.start(100)

    def btnCoatFaceClicked(self):
        ''' Get path for coating the upside face '''
        self.robpath.init_coating()
        self.plot.drawWorkingArea()
        self.plot.drawPath(self.robpath.path, self.robpath.part.color)
        self.btnSaveRapid.setEnabled(True)

    def btnSaveRapidClicked(self):
        filename = QtGui.QFileDialog.getSaveFileName(
            None, 'Save file', None,
            'All Files (*);;ABB Files (*.mod)')
        if len(filename)==0:
            print 'Type a file name'
            return
        if filename.split('.')[-1] != 'mod':
            filename = filename + '.mod'

        self.rapid.dynamic_params = self.robpath.dynamic_params
        self.rapid.params_group = self.robpath.params_group
        #if os.path.exists(self.dirname + '/base_frame.json'):
        #    self.robpath.load_base_frame(self.dirname + '/base_frame.json')
        self.robpath.path = self.robpath.transform_path(self.robpath.path)
        routine = self.rapid.path2rapid_beta(self.robpath.path)
        self.rapid.save_file(filename, routine)
        filename = filename.split('.')[0] + '.xml'
        self.robpath.save_xml(filename, self.robpath.path)
        #self.rapid.upload_file(filename, directory)
        QtGui.QMessageBox.information(
            self, "Export information", "Routine exported to the robot.")

    def btnQuitClicked(self):
        QtCore.QCoreApplication.instance().quit()


if __name__ == "__main__":
    # Don't create a new QApplication, it would unhook the Events
    # set by Traits on the existing QApplication. Simply use the
    # '.instance()' method to retrieve the existing one.
    app = QtGui.QApplication.instance()
    robpath = RobPathUI()
    robpath.show()
    app.exec_()

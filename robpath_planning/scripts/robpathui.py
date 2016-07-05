#!/usr/bin/env python
import os
os.environ['QT_API'] = 'pyqt'
os.environ['ETS_TOOLKIT'] = 'qt4'

# To be able to use PySide or PyQt4 and not run in conflicts with traits,
# we need to import QtGui and QtCore from pyface.qt
from pyface.qt import QtGui, QtCore

#from mayavi import mlab
from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item
from mayavi.core.ui.api import MlabSceneModel, SceneEditor
from tvtk.pyface.api import Scene

#from PyQt4 import QtGui, QtCore, uic
from PyQt4 import uic

import numpy as np

from robpath import RobPath
from robpath.mlabplot import MPlot3D


class Visualization(HasTraits):
    scene = Instance(MlabSceneModel, ())

    @on_trait_change('scene.activated')
    def update_plot(self):
        # This function is called when the view is opened.
        self.scene.mlab.view(0, 65, 150)
        self.scene.background = (0.2, 0.2, 0.2)
        pass

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

    def drawPath(self, path):
        self.mlab.draw_path(path)

    def drawSlice(self, slices, path):
        self.mlab.draw_slice(slices[-1])
        #self.plot.mlab.draw_path(tool_path)
        self.mlab.draw_path(path)


class RobPathUI(QtGui.QMainWindow):
    def __init__(self):
        super(RobPathUI, self).__init__()
        path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        uic.loadUi(os.path.join(path, 'resources', 'robpath.ui'), self)

        self.plot = QMayavi()
        self.boxPlot.addWidget(self.plot)
        self.plot.drawWorkingArea()

        self.btnLoadMesh.clicked.connect(self.btnLoadMeshClicked)
        self.btnProcessMesh.clicked.connect(self.btnProcessMeshClicked)
        self.btnProcessContours.clicked.connect(self.btnProcessContoursClicked)
        self.btnSaveRapid.clicked.connect(self.btnSaveRapidClicked)

        self.sbPositionX.valueChanged.connect(self.changePosition)
        self.sbPositionY.valueChanged.connect(self.changePosition)
        self.sbPositionZ.valueChanged.connect(self.changePosition)

        self.sbSizeX.valueChanged.connect(self.changeSize)
        self.sbSizeY.valueChanged.connect(self.changeSize)
        self.sbSizeZ.valueChanged.connect(self.changeSize)

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        self.processing = False
        self.timer = QtCore.QTimer(self.plot)
        self.timer.timeout.connect(self.updateProcess)

        self.robpath = RobPath()
        self.filled = True

    def changePosition(self):
        x = self.sbPositionX.value()
        y = self.sbPositionY.value()
        z = self.sbPositionZ.value()
        self.robpath.translate_mesh(np.float32([x, y, z]))
        self.plot.drawMesh(self.robpath.mesh)

    def changeSize(self):
        sx = self.sbSizeX.value() + 0.001
        sy = self.sbSizeY.value() + 0.001
        sz = self.sbSizeZ.value() + 0.001
        self.robpath.resize_mesh(np.float32([sx, sy, sz]))
        self.changePosition()

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

    def updateProcess(self):
        if self.robpath.k < len(self.robpath.levels):
            self.robpath.update_process(
                filled=self.filled, contour=not self.filled)
            #self.plot.drawSlice(self.robpath.slices, self.robpath.path)
            self.plot.drawPath(self.robpath.path)
            self.plot.progress.setValue(100.0 * self.robpath.k / len(self.robpath.levels))
            self.btnSaveRapid.setEnabled(False)
        else:
            self.processing = False
            self.timer.stop()
            self.btnSaveRapid.setEnabled(True)

    def blockSignals(self, value):
        self.sbPositionX.blockSignals(value)
        self.sbPositionY.blockSignals(value)
        self.sbPositionZ.blockSignals(value)
        self.sbSizeX.blockSignals(value)
        self.sbSizeY.blockSignals(value)
        self.sbSizeZ.blockSignals(value)

    def btnLoadMeshClicked(self):
        self.blockSignals(True)
        try:
            filename = QtGui.QFileDialog.getOpenFileName(
                self.plot, 'Open file', './', 'Mesh Files (*.stl)')
            print filename
            self.setWindowTitle('Mesh Viewer: %s' % filename)
            self.robpath.load_mesh(filename)
            # -----
            # TODO: Change bpoints.
            self.updatePosition(self.robpath.mesh.bpoint1)
            self.updateSize(self.robpath.mesh.bpoint2 - self.robpath.mesh.bpoint1)
            self.lblInfo.setText('Info:\n')
            # -----
            self.plot.drawMesh(self.robpath.mesh)
            self.btnProcessMesh.setEnabled(True)
            self.btnProcessContours.setEnabled(True)
        except:
            pass
        self.blockSignals(False)
        self.plot.drawMesh(self.robpath.mesh)

    def update_parameters(self):
        height = self.sbHeight.value() + 0.00001
        width = self.sbWidth.value() + 0.00001
        overlap = 0.01 * self.sbOverlap.value()
        self.robpath.set_track(height, width, overlap)

        speed = self.sbSpeed.value()
        power = self.sbPower.value()
        self.robpath.set_speed(speed)
        self.robpath.set_power(power)

        carrier_gas = self.sbCarrier.value()
        stirrer = self.sbStirrer.value()
        turntable = self.sbTurntable.value()
        self.robpath.set_powder(carrier_gas, stirrer, turntable)

    def __process_shape(self):
        if self.processing:
            self.timer.stop()
            self.processing = False
        else:
            self.plot.drawWorkingArea()

            self.update_parameters()
            self.robpath.init_process()

            self.processing = True
            self.timer.start(100)

    def btnProcessMeshClicked(self):
        self.filled = True
        self.__process_shape()

    def btnProcessContoursClicked(self):
        self.filled = False
        self.__process_shape()

    def btnSaveRapidClicked(self):
        self.robpath.save_rapid()
        QtGui.QMessageBox.information(self, "Export information", "Routine exported to the robot.")

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

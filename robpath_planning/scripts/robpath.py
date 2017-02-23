#!/usr/bin/env python
import os
import numpy as np
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

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        self.dirname = path
        self.selected = None
        self.processing = False
        self.timer = QtCore.QTimer(self.plot)
        self.timer.timeout.connect(self.updateProcessing)

        self.robpath = RobPath()
        self.rapid = Rapid()

    def blockSignals(self, value):
        self.sbPositionX.blockSignals(value)
        self.sbPositionY.blockSignals(value)
        self.sbPositionZ.blockSignals(value)
        self.sbSizeX.blockSignals(value)
        self.sbSizeY.blockSignals(value)
        self.sbSizeZ.blockSignals(value)

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
        self.robpath.part.set_process(speed, power, focus)
        self.rapid.set_process(speed, power)

    def changePowder(self):
        carrier = self.sbCarrier.value()
        stirrer = self.sbStirrer.value()
        turntable = self.sbTurntable.value()
        self.robpath.part.set_powder(carrier, stirrer, turntable)
        self.rapid.set_powder(carrier, stirrer, turntable)

    def changeFilling(self):
        self.robpath.part.filling = self.sbFilling.value()

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

    def updateFilling(self, filling):
        self.sbFilling.setValue(filling)

    def btnLoadMeshClicked(self):
        try:
            filename = QtGui.QFileDialog.getOpenFileName(
                self.plot, 'Open file', self.dirname, 'Mesh Files (*.stl)')
            if filename:
                self.dirname = os.path.dirname(filename)
                self.robpath.load_mesh(filename)
                self.setWindowTitle('Mesh Viewer: %s' % filename)
                self.btnSelectMesh.addItems([self.robpath.name])
                self.updateMeshData(self.robpath.name)
                self.btnProcessMesh.setEnabled(True)
                self.btnProcessContours.setEnabled(True)
                self.robpath.part.filling = self.sbFilling.value()
        except:
            pass
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
        self.updateFilling(self.robpath.part.filling)
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
        if self.robpath.k < len(self.robpath.levels):
            self.robpath.update_process(filled=self.chbFilled.isChecked(),
                                        contour=self.chbContour.isChecked())
            #self.plot.drawSlice(self.robpath.slices, self.robpath.path)
            self.plot.drawPath(self.robpath.path, self.robpath.part.color)
            self.plot.progress.setValue(100.0 * self.robpath.k / len(self.robpath.levels))
            self.btnSaveRapid.setEnabled(False)
        else:
            self.processing = False
            self.timer.stop()
            self.btnSaveRapid.setEnabled(True)

    def btnProcessMeshClicked(self):
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

    def btnSaveRapidClicked(self):
        filename = 'robpath.mod'
        directory = '../../AIMEN'
        routine = self.rapid.path2rapid_beta(self.robpath.path)
        self.rapid.save_file(filename, routine)
        self.rapid.upload_file(filename, directory)
        print routine
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

#!/usr/bin/env python
import os
import sys
import rospy
import rospkg
import datetime
import numpy as np

from visualization_msgs.msg import MarkerArray
from markers import PartMarkers

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from planning.robpath import RobPath


path = rospkg.RosPack().get_path('robpath_part')


class QtPart(QtGui.QWidget):
    accepted = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'part.ui'), self)

        self.pub_marker_array = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=10)

        self.btnLoad.clicked.connect(self.btnLoadClicked)
        self.btnProcessMesh.clicked.connect(self.btnProcessMeshClicked)
        self.btnLayers.clicked.connect(self.btnLayersClicked)
        self.btnAcceptPath.clicked.connect(self.btnAcceptPathClicked)

        self.sbStart.valueChanged.connect(self.changeLayers)
        self.sbStop.valueChanged.connect(self.changeLayers)

        self.sbPositionX.valueChanged.connect(self.changePosition)
        self.sbPositionY.valueChanged.connect(self.changePosition)
        self.sbPositionZ.valueChanged.connect(self.changePosition)

        self.sbSizeX.valueChanged.connect(self.changeSize)
        self.sbSizeY.valueChanged.connect(self.changeSize)
        self.sbSizeZ.valueChanged.connect(self.changeSize)

        self.processing = False
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateProcess)

        self.robpath = RobPath()

    def btnLoadClicked(self):
        self.blockSignals(True)
        self.processing = False

        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Open file', os.path.join(path, 'data'),
            'Mesh Files (*.stl)')[0]
        self.setWindowTitle(filename)
        self.robpath.load_mesh(filename)
        self.dirname = os.path.dirname(filename)
        self.updateParameters()

        self.updatePosition(self.robpath.part.position)
        self.updateSize(self.robpath.part.size)

        self.part_markers = PartMarkers()
        self.part_markers.set_mesh(self.robpath.part)
        self.pub_marker_array.publish(self.part_markers.marker_array)

        self.updateLayers()
        self.blockSignals(False)
        self.updatePosition((10, 10, 100))

    def updateParameters(self):
        height = self.sbHeight.value() + 0.00001
        width = self.sbWidth.value() + 0.00001
        overlap = 0.01 * self.sbOverlap.value()
        self.robpath.part.set_track(height, width, overlap)
        self.robpath.part.set_process(rospy.get_param('/process/speed'),
                                      rospy.get_param('/process/power'),
                                      rospy.get_param('/process/focus'))

    def updateLayers(self):
        self.npoints = 0
        levels = self.robpath.init_process()
        self.sbStart.setValue(self.robpath.k)
        self.sbStop.setValue(len(levels))

    def updateProcess(self):
        if self.robpath.k < len(self.robpath.levels):
            self.robpath.update_process(filled=self.chbFill.isChecked(),
                                        contour=self.chbContour.isChecked())
            self.sbStart.setValue(self.robpath.k)
            self.part_markers.set_path(self.robpath.path[self.npoints:-1])
            self.npoints = len(self.robpath.path)
            self.pub_marker_array.publish(self.part_markers.marker_array)
        else:
            self.processing = False
            self.timer.stop()
        self.updateInfo()

    def updateInfo(self):
        length = self.robpath.planning.path_length(self.robpath.path)
        time = self.robpath.planning.path_time(
            length, self.robpath.part.speed, 50)
        #self.lblInfo.setText("Estimated time:s %.1f s" % (time))
        self.lblInfo.setText("Estimated time:   %s" % (
            str(datetime.timedelta(seconds=int(time)))))

    def btnProcessMeshClicked(self):
        if self.processing:
            self.processing = False
            self.timer.stop()
        else:
            self.updateParameters()
            self.updateLayers()
            self.processing = True
            self.timer.start(100)

    def btnLayersClicked(self):
        if self.processing:
            self.updateParameters()
            self.updateProcess()
        else:
            self.updateParameters()
            self.updateLayers()
            self.processing = True

    def btnAcceptPathClicked(self):
        self.robpath.load_base_frame(self.dirname + '/base_frame.json')
        print self.dirname + '/base_frame.json'
        self.robpath.path = self.robpath.transform_path(self.robpath.path)
        self.accepted.emit(self.robpath.path)

    def changeLayers(self):
        start = self.sbStart.value()
        stop = self.sbStop.value()
        self.robpath.k = int(start)

    def updatePosition(self, position):
        x, y, z = position
        self.sbPositionX.setValue(x)
        self.sbPositionY.setValue(y)
        self.sbPositionZ.setValue(z)

    def changePosition(self):
        x = self.sbPositionX.value()
        y = self.sbPositionY.value()
        z = self.sbPositionZ.value()
        self.updatePart(position=(x, y, z))

    def updateSize(self, size):
        sx, sy, sz = size
        self.sbSizeX.setValue(sx)
        self.sbSizeY.setValue(sy)
        self.sbSizeZ.setValue(sz)

    def changeSize(self):
        sx = self.sbSizeX.value() + 0.001
        sy = self.sbSizeY.value() + 0.001
        sz = self.sbSizeZ.value() + 0.001
        self.updatePart(size=(sx, sy, sz))

    def updatePart(self, position=None, size=None):
        if position is not None:
            self.robpath.part.translate(np.float32(position))
            self.part_markers.set_mesh(self.robpath.part)
        if size is not None:
            self.robpath.part.resize_mesh(np.float32(size))
            self.part_markers.set_mesh(self.robpath.part)
        self.pub_marker_array.publish(self.part_markers.marker_array)

    def blockSignals(self, value):
        self.sbPositionX.blockSignals(value)
        self.sbPositionY.blockSignals(value)
        self.sbPositionZ.blockSignals(value)
        self.sbSizeX.blockSignals(value)
        self.sbSizeY.blockSignals(value)
        self.sbSizeZ.blockSignals(value)


if __name__ == "__main__":
    rospy.init_node('part_panel')

    app = QtGui.QApplication(sys.argv)
    qt_part = QtPart()
    qt_part.show()
    app.exec_()

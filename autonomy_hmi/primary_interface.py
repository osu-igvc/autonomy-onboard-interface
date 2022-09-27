from symtable import Symbol
import sys

import matplotlib as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

import numpy as np

from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6 import uic

from PyQt6.QtWidgets import QMainWindow, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QApplication
from PyQt6.QtCore     import QSize
from PyQt6.QtGui      import QIcon, QAction

# ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import threading
import time

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)


class ImageReceiver(Node):
    def __init__(self, image_topic, parent=None):
        super().__init__('image_receiver')

        self.subscription = self.create_subscription(Image, image_topic, self.imageCallback, 10)

        self.bridge = CvBridge()

        self.parent = parent

        #self.spin_thread = threading.Thread(target=self.spinThread)
        #self.spin_thread.start()

        if self.parent is not None:
            self.parent.on_laneImageReceive()
            self.get_logger().info('TESTING')


    def imageCallback(self, msg):
        self.get_logger().info('Received image')
        frame = self.bridge.imgmsg_to_cv2(msg)
        self.latest_frame = frame

        if self.parent is not None:
            self.parent.on_laneImageReceive()

    def spinThread(self):
        rclpy.spin(self)
        rclpy.shutdown()

    
    
class UI(QMainWindow):
    def __init__(self, dbwEnabled):
        super(UI, self).__init__()
        uic.loadUi("install/autonomy_hmi/share/ament_index/resource_index/packages/window.ui", self)
        self.show()
        rclpy.init()

        self.dbwEnabled = dbwEnabled

        self.laneDetectionImages = ImageReceiver('/cameras/front_lane_markings')
        
        self.ros_thread = threading.Thread(target=self.spinROSThread)
        self.ros_thread.start()
        self.imageCount = 0

    def __del__(self):
        print('destroying window')
        rclpy.shutdown()
        self.ros_thread.join()

    def spinROSThread(self):
        rclpy.spin(self.laneDetectionImages)
        
        
    @QtCore.pyqtSlot()
    def on_actionNew_triggered(self):
        print("New")
        
    @QtCore.pyqtSlot()
    def on_actionOpen_triggered(self):
        print("Open")
        
    @QtCore.pyqtSlot()
    def on_actionExit_triggered(self):
        print("Exit")
        
    @QtCore.pyqtSlot()
    def on_pushButton_clicked(self):
        self.dbwEnabled = ~self.dbwEnabled
        #self.updateDbwEnabled()

        
    def on_listWidget_itemClicked(self, item: QtWidgets.QListWidgetItem):
        ab = QtWidgets.QStackedWidget()
        if item.data(0) == "Main":
            self.stackedWidget.setCurrentIndex(0)
        elif item.data(0) == "Velocity vs Time":
            self.stackedWidget.setCurrentIndex(1)
        elif item.data(0) == "Brakes vs Time":
            self.stackedWidget.setCurrentIndex(2)
        elif item.data(0) == "Debug Stream":
            self.stackedWidget.setCurrentIndex(3)
            
    # def updateDbwEnabled(self):          
    #     if self.dbwEnabled == False:
    #         self.label_5.setText("DISABLED")
    #         self.label_5.setStyleSheet("background-color: red")
    #     else:
    #         self.label_5.setText("ENABLED")
    #         self.label_5.setStyleSheet("background-color: green")


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = UI(dbwEnabled=False)

    with open('install/autonomy_hmi/share/ament_index/resource_index/packages/style.qss', "r") as f:
        _style = f.read()
        window.setStyleSheet(_style)
    
    app.exec()
        
if __name__ == "__main__":
    main()
from concurrent.futures import thread
from symtable import Symbol
import sys
from tkinter import W

import matplotlib as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

import numpy as np

from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6 import uic

from PyQt6.QtWidgets import QMainWindow, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QApplication
from PyQt6.QtCore     import QSize, Qt
from PyQt6.QtGui      import QIcon, QAction, QPixmap

# ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import threading
import time



# Very useful tutorial on image conversion from OpenCV to PyQt
# https://www.imagetracking.org.uk/2020/12/displaying-opencv-images-in-pyqt/


   
class UI(QMainWindow):
    def __init__(self, dbwEnabled):
        super(UI, self).__init__()
        uic.loadUi("install/autonomy_hmi/share/ament_index/resource_index/packages/window.ui", self)
        self.show()
        self.dbwEnabled = dbwEnabled

        self.ros_node = GUINode(parentGUI=self)
        self.ros_thread = threading.Thread(target=self.rosSpin)
        self.ros_thread.start()

    def rosSpin(self):
        self.ros_node.get_logger().info('spinning up ros')

        rclpy.spin(self.ros_node)
    
    def close_nodes(self):
        self.ros_node.get_logger().info('shutting down ros')
        self.ros_node.destroy_node()

    
    def setSwitchableImage(self, frame):

        scaled_frame = frame.scaled(self.switchable_display.width(), self.switchable_display.height(), Qt.AspectRatioMode.KeepAspectRatio)
        self.switchable_display.setPixmap(QPixmap.fromImage(scaled_frame))
        
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


class GUINode(Node):

    def __init__(self, parentGUI = None):

        super().__init__('gui_node')
        self.parentGUI = parentGUI
        self.numCount = 0
        self.subscription = self.create_subscription(Image, '/cameras/front_lane_markings', self.imageCallback, 10)
        self.bridge = CvBridge()

    def imageCallback(self, msg):
        self.get_logger().info('Received image')
        self.numCount += 1

        cv_img = self.bridge.imgmsg_to_cv2(msg)
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w

        qt_img = QtGui.QImage(rgb_img.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        
        if self.parentGUI is not None:
            #self.parentGUI.newImage('image #' + str(self.numCount))
            self.parentGUI.setSwitchableImage(qt_img)

        
    
 

def main():
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)
    window = UI(dbwEnabled=False)

    with open('install/autonomy_hmi/share/ament_index/resource_index/packages/style.qss', "r") as f:
        _style = f.read()
        window.setStyleSheet(_style)

    # Do we get out of here?
    app.exec()

    window.close_nodes()

    rclpy.shutdown()
    
        
if __name__ == "__main__":
    main()
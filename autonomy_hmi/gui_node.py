
# Python Imports
from concurrent.futures import thread
from symtable import Symbol
import sys
from tkinter import W
from typing import Callable
from urllib.request import Request

# Data manipulation
import matplotlib as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import numpy as np
import threading
import time
from enum import Enum

# PYQT Imports
from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6 import uic
from PyQt6.QtWidgets import QMainWindow, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QApplication
from PyQt6.QtCore     import QSize, Qt
from PyQt6.QtGui      import QIcon, QAction, QPixmap
from requests import request

# ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.action import ActionClient
from std_msgs.msg import Bool

# IGVC Autonomy Imports
from autonomy_interfaces.action import RequestEnableDisable



# Very useful tutorial on image conversion from OpenCV to PyQt
# https://www.imagetracking.org.uk/2020/12/displaying-opencv-images-in-pyqt/



   
class UI(QMainWindow):
    def __init__(self):
        super(UI, self).__init__()
        uic.loadUi("install/autonomy_hmi/share/ament_index/resource_index/packages/window.ui", self)
        self.show()

        self.ros_node = GUINode(parentGUI=self)
        self.ros_thread = threading.Thread(target=self.rosSpin)
        self.ros_thread.start()

        self.enableBtn = EnableButton(self, self.enable_btn)

    # ============================= ROS STARTUP =============================

    def rosSpin(self):
        self.ros_node.get_logger().info('Spinning up ros')
        rclpy.spin(self.ros_node)
    
    def close_node(self):
        self.ros_node.get_logger().info('Shutting down ros')
        self.ros_node.destroy_node()

    
    # ============================= Switchable views =============================
    def setSwitchableImage(self, frame):
        #scaled_frame = frame.scaled(self.switchable_display.width(), self.switchable_display.height(), Qt.AspectRatioMode.KeepAspectRatio)
        #self.switchable_display.setPixmap(QPixmap.fromImage(scaled_frame))
        pass


    # ============================= MISC =============================

    @QtCore.pyqtSlot()
    def on_actionNew_triggered(self):
        print("New")
        
    @QtCore.pyqtSlot()
    def on_actionOpen_triggered(self):
        print("Open")
        
    @QtCore.pyqtSlot()
    def on_actionExit_triggered(self):
        print("Exit")
        
    # Handles switching tabs using list on left-hand side of GUI
    def on_sidebar_list_itemClicked(self, item: QtWidgets.QListWidgetItem):
        ab = QtWidgets.QStackedWidget()
        if item.data(0) == "Dashboard":  
            self.stackedWidget.setCurrentIndex(0)
        elif item.data(0) == "Camera Tuning":
            self.stackedWidget.setCurrentIndex(1)
        elif item.data(0) == "WIP":
            self.stackedWidget.setCurrentIndex(2)
        elif item.data(0) == "WIP":
            self.stackedWidget.setCurrentIndex(3)

        self.ros_node.get_logger().warn("SIDEBAR CLICKED: " + str(item.data(0)))


    # ============================= AUTONOMY ENABLE BUTTON =============================

    @QtCore.pyqtSlot()
    def on_enable_btn_clicked(self):
        self.ros_node.get_logger().warn("ENABLE BTN PRESSED")
        self.enableBtn.pressed()
        # self.indicator_gear_label.setText(str(result))

        # if result:
        #     self.enable_btn.setStyleSheet('background-color: green')
        # else:
        #     pass


class GUINode(Node):

    def __init__(self, parentGUI: UI = None):

        super().__init__('gui_node')
        self.parentGUI = parentGUI
        self.numCount = 0
        self.subscription = self.create_subscription(Image, '/cameras/front_lane_markings', self.imageCallback, 10)
        self.bridge = CvBridge()

        self.enable_server_client = ActionClient(self, RequestEnableDisable, 'SystemEnable')

        # Health subs
        self.health_server_sub = self.create_subscription(Bool, '/health/system_health', self.systemHealthCallback, 10)
        self.enable_status_sub = self.create_subscription(Bool, '/health/enable_status', self.systemEnableCallback, 10)
        self._systemHealthStatus = False

    def systemHealthCallback(self, msg):
        self._systemHealthStatus = msg.data

        if self.parentGUI is not None:
            self.parentGUI.enableBtn.healthStateUpdated(self._systemHealthStatus)

    def systemEnableCallback(self, msg):
        pass

    def imageCallback(self, msg):
        #self.get_logger().info('Received image')
        self.numCount += 1

        cv_img = self.bridge.imgmsg_to_cv2(msg)
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w

        qt_img = QtGui.QImage(rgb_img.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        
        if self.parentGUI is not None:
            #self.parentGUI.newImage('image #' + str(self.numCount))
            self.parentGUI.setSwitchableImage(qt_img)

    
    def updateEnableStatus(self, new_status, feedback_callback: Callable, goal_response_callback: Callable):
        enable_msg = RequestEnableDisable.Goal()
        enable_msg.set_enabled = new_status

        self.enable_server_client.wait_for_server()

        # Not sure if correct
        self.__send_goal_future = self.enable_server_client.send_goal_async(enable_msg, feedback_callback)
        self.__send_goal_future.add_done_callback(goal_response_callback)
    

class EnableButton():

    class EnableButtonState(Enum):
        disabled_not_healthy = 1,
        disabled_healthy = 2,
        waiting = 3,
        enabled = 4

    def __init__(self, parent: UI, button: QPushButton):
        self.btn = button
        self.healthState = False
        self.buttonState = self.EnableButtonState.disabled_not_healthy
        self.nextButtonState = self.EnableButtonState.disabled_not_healthy
        self.parent = parent

        self.updateState()

    def getState(self):
        return self.buttonState

    def updateState(self):
        if self.buttonState == self.EnableButtonState.disabled_not_healthy:
            self.btn.setText('DISABLED')
            self.btn.setDisabled(True)
            self.btn.setStyleSheet('background-color: grey')

        elif self.buttonState == self.EnableButtonState.disabled_healthy:
            self.btn.setText('DISABLED')
            self.btn.setEnabled(True)
            self.btn.setStyleSheet('background-color: yellow')

        elif self.buttonState == self.EnableButtonState.waiting:
            self.btn.setText('WAITING')
            self.btn.setStyleSheet('background-color: orange')
        
        elif self.buttonState == self.EnableButtonState.enabled:
            self.btn.setText('ENABLED')
            self.btn.setStyleSheet('background-color: green')


    def healthStateUpdated(self, newState: bool):
        self.healthState = newState

        if self.healthState:
            if self.buttonState == self.EnableButtonState.disabled_not_healthy:
                self.buttonState = self.EnableButtonState.disabled_healthy
        else:
            self.buttonState == self.EnableButtonState.waiting
            self.requestDisable()

        self.updateState()

    def pressed(self):
        if self.buttonState == self.EnableButtonState.disabled_not_healthy:
            self.parent.ros_node.get_logger().info('DISABLED - NOT HEALTHY')

        elif self.buttonState == self.EnableButtonState.disabled_healthy:
            self.parent.ros_node.get_logger().info('REQUESTING ENABLE')
            self.requestEnable()

        elif self.buttonState == self.EnableButtonState.waiting:
            self.parent.ros_node.get_logger().info('WAITING ON ENABLE RESPONSE')

        elif self.buttonState == self.EnableButtonState.enabled:
            self.parent.ros_node.get_logger().info('REQUESTING DISABLE')
            self.requestDisable()

        else: # If not in known state, set to one.
            self.buttonState = self.EnableButtonState.disabled_not_healthy

    def requestEnable(self):
        #pass
        # Ask: Are you sure you want to enable autonomy?
        self.nextButtonState = self.EnableButtonState.enabled
        self.parent.ros_node.updateEnableStatus(True, self.goalFeedback, self.goalResponse)

    def requestDisable(self):
        #pass
        self.nextButtonState = self.EnableButtonState.disabled_healthy
        self.parent.ros_node.updateEnableStatus(False, self.goalFeedback, self.goalResponse)
        # Ask: are you sure you want to disable autonomy?

    def goalFeedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.parent.ros_node.get_logger().info('FEEDBACK RECEIVED')

    def goalResponse(self, response_msg):
        rs = response_msg.result()

        if not rs.accepted:
            self.nextButtonState = self.buttonState
            self.parent.ros_node.get_logger().info('RESULT NOT ACCEPTED')
            return
        
        self.parent.ros_node.get_logger().info('RESULT ACCEPTED')
        self.future_result = rs.get_result_async()
        self.future_result.add_done_callback(self.goalResultCallback)

    def goalResultCallback(self, result_msg):
        result = result_msg.result().result
        self.parent.ros_node.get_logger().info('RESULT: {0}'.format(result.new_enable_status))
        self.buttonState = self.nextButtonState
        self.updateState()




def main():
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)
    window = UI()

    with open('install/autonomy_hmi/share/ament_index/resource_index/packages/style.qss', "r") as f:
        _style = f.read()
        window.setStyleSheet(_style)

    app.exec()

    window.close_node()

    rclpy.shutdown()
    
        
if __name__ == "__main__":
    main()
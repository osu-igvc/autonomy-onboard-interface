from symtable import Symbol
import sys
import random

import matplotlib as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

import numpy as np

from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6 import uic

from PyQt6.QtWidgets import QMainWindow, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QApplication
from PyQt6.QtCore     import QSize
from PyQt6.QtGui      import QIcon, QAction

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
    
    
class UI(QMainWindow):
    def __init__(self, dbwEnabled):
        super(UI, self).__init__()
        uic.loadUi("window.ui", self)
        self.show()
        
        self.dbwEnabled = dbwEnabled
        self.updateDbwEnabled()
        
        x = range(0,10)
        y = [2,8,6,8,6,11,14,13,18,19]
        
        sc = MplCanvas(self, width=5, height=4, dpi=100)
        sc.axes.plot(x,y)
        
        self.stackedWidget.addWidget(sc)
        
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
        self.updateDbwEnabled()

        
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
            
    def updateDbwEnabled(self):          
        if self.dbwEnabled == False:
            self.label_5.setText("DISABLED")
            self.label_5.setStyleSheet("background-color: red")
        else:
            self.label_5.setText("ENABLED")
            self.label_5.setStyleSheet("background-color: green")
        
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = UI(dbwEnabled=False)

    with open("style.qss", "r") as f:
        _style = f.read()
        window.setStyleSheet(_style)
    
    app.exec()
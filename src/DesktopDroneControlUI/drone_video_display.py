#!/usr/bin/env python

#Inherited code from 
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib#; roslib.load_manifest('ardrone_tutorials')
import rospy
import os

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image         # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui


# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 8 # the radius of the circle drawn when a tag is detected

class ClickableLabel(QtGui.QLabel):
    '''Normal label, but emits an event if the label is left-clicked''' 
    
    signalClicked = QtCore.Signal()    # emitted whenever this label is left-clicked
    
    def __init__(self, id_num, parent=None):
        super(ClickableLabel, self).__init__(parent)
        self.setStyleSheet('''
        QLabel:hover {
        background-color: red;
        }
        ''')
        self.id_num = id_num
        self.parent = parent
        # self.setFixedSize(self.sizeHint())

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            print "clicked " + str(self.id_num)
            self.parent.clickedLabel = self.id_num
            event.ignore()
            # self.signalClicked.emit()
        return True

class CentralWidget(QtGui.QWidget):

    def __init__(self):
        super(CentralWidget, self).__init__()
        self.imageBox = QtGui.QLabel(self)
        self.clickedLabel = -1
        self.initSidePane()

    def setZigzagLayout(self):
        self.lbls[0].move(640 + 160, 0*95)
        self.lbls[1].move(640 + 400, 0*95)
        self.lbls[2].move(640 + 160, 1*95)
        self.lbls[3].move(640 + 400, 1*95)
        self.lbls[4].move(640 + 160, 2*95)
        self.lbls[5].move(640 + 400, 2*95)
        self.lbls[6].move(640 + 160, 3*95)
        self.lbls[7].move(640 + 400, 3*95)

    def setDefaultLayout(self):
        self.lbls[0].move(640 + 0*80 + 10, 2.5*55)
        self.lbls[1].move(640 + 1*80 + 10, 1*55)
        self.lbls[2].move(640 + 3*80 + 10, 0*55)
        self.lbls[3].move(640 + 5*80 + 10, 1*55)
        self.lbls[4].move(640 + 6*80 + 10, 2.5*55)
        self.lbls[5].move(640 + 5*80 + 10, 4*55)
        self.lbls[6].move(640 + 3*80 + 10, 5*55)
        self.lbls[7].move(640 + 1*80 + 10, 4*55)


    def initSidePane(self):
        """ Initialize the side pane for user interaction.
        """
        pixmap = QtGui.QPixmap("blank_image.png")
        self.imageBox.setPixmap(pixmap)

        self.lbls = []
        for i in xrange(8):
            lbl = ClickableLabel(i, self)
            lbl.setPixmap(pixmap.scaledToHeight(80))
            self.lbls.append(lbl)

        self.setDefaultLayout()

class DroneVideoDisplay(QtGui.QMainWindow):
    StatusMessages = {
        DroneStatus.Emergency : 'Emergency',
        DroneStatus.Inited    : 'Initialized',
        DroneStatus.Landed    : 'Landed',
        DroneStatus.Flying    : 'Flying',
        DroneStatus.Hovering  : 'Hovering',
        DroneStatus.Test      : 'Test (?)',
        DroneStatus.TakingOff : 'Taking Off',
        DroneStatus.GotoHover : 'Going to Hover Mode',
        DroneStatus.Landing   : 'Landing',
        DroneStatus.Looping   : 'Looping (?)'
        }
    DisconnectedMessage = 'Disconnected'
    UnknownMessage = 'Unknown Status'
    
    def __init__(self):
        # Construct the parent class
        super(DroneVideoDisplay, self).__init__()

        # Setup our very basic GUI - a label which fills the whole window and holds our image
        self.setWindowTitle('AR.Drone Video Feed')
        self.setGeometry(40, 40, 1290, 370)
        self.centralWidget = CentralWidget()
        self.setCentralWidget(self.centralWidget)

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
        
        # Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
        self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
        
        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()

        self.tags = []
        self.tagLock = Lock()

        self.circles = []
        self.small_circles = []
        self.points = []
        self.qimages = []
        self.initQimages()
        
        # Holds the status message to be displayed on the next GUI update
        self.statusMessage = ''

        # Tracks whether we have received data since the last connection check
        # This works because data comes in at 50Hz but we're checking for a connection at 4Hz
        self.communicationSinceTimer = False
        self.connected = False

        # A timer to check whether we're still connected
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    def initQimages(self):
        for i in xrange(8):
            pixmap = QtGui.QPixmap("blank_image.png")
            temp = {'ToDraw':False, 'image': pixmap}
            self.qimages.append(temp)

    def resetQimages(self):
        for i in xrange(len(self.qimages)):
            pixmap = QtGui.QPixmap("blank_image.png")
            self.qimages[i]['ToDraw'] = True
            self.qimages[i]['image'] = pixmap

    def DrawLabels(self):
        for i in xrange(8):
            if self.qimages[i]['ToDraw'] == True:
                self.centralWidget.lbls[i].setPixmap(\
                        self.qimages[i]['image'].scaledToHeight(80))
                self.qimages[i]['ToDraw'] = False

    def DrawCircles(self):
        self.tagLock.acquire()
        try:
            painter = QtGui.QPainter()
            painter.begin(self.qimage)
            painter.setPen(QtGui.QColor(0,255,0,95))
            painter.setBrush(QtGui.QColor(0,255,0,95))
            # for (x,y,d) in self.circles:
            #     r = QtCore.QRectF((x*self.qimage.width())/1000-DETECT_RADIUS,(y*self.qimage.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
            #     painter.drawEllipse(r)
            #     painter.drawText((x*self.qimage.width())/1000+DETECT_RADIUS,(y*self.qimage.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
            for (x, y, d, color) in self.circles:
                painter.setPen(QtGui.QColor(*color))
                painter.setBrush(QtGui.QColor(*color))
                qpoint = QtCore.QPoint(x, y)
                painter.drawEllipse(qpoint, d, d)
            for (x, y, d, color) in self.small_circles:
                painter.setPen(QtGui.QColor(*color))
                painter.setBrush(QtGui.QColor(*color))
                qpoint = QtCore.QPoint(x, y)
                painter.drawEllipse(qpoint, d, d)
            painter.end()
        finally:
            self.tagLock.release()

    def DrawPoints(self):
        self.tagLock.acquire()
        try:
            painter = QtGui.QPainter()
            painter.begin(self.qimage)
            pen = QtGui.QPen(QtCore.Qt.red, 5, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.MiterJoin)
            painter.setPen(pen)
            painter.setBrush(QtGui.QColor(255,0,0))
            painter.drawPoints(self.points)
            painter.end()
        finally:
            self.tagLock.release()

    # Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    def RedrawCallback(self):
        if self.image is not None:
            # We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
            self.imageLock.acquire()
            try:            
                    # Convert the ROS image into a QImage which we can display
                    self.qimage = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))

                    if len(self.points) > 0:
                        self.DrawPoints()
                    if (len(self.circles) + len(self.small_circles)) > 0:
                        self.DrawCircles()
                    if len(self.tags) > 0:
                        self.tagLock.acquire()
                        try:
                            painter = QtGui.QPainter()
                            painter.begin(self.qimage)
                            painter.setPen(QtGui.QColor(0,255,0))
                            painter.setBrush(QtGui.QColor(0,255,0))
                            for (x,y,d) in self.tags:
                                r = QtCore.QRectF((x*self.qimage.width())/1000-DETECT_RADIUS,(y*self.qimage.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
                                painter.drawEllipse(r)
                                painter.drawText((x*self.qimage.width())/1000+DETECT_RADIUS,(y*self.qimage.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
                            painter.end()
                        finally:
                            self.tagLock.release()
            finally:
                self.imageLock.release()

            # We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
            # self.resize(self.qimage.width(),self.qimage.height())
            self.centralWidget.imageBox.setPixmap(self.qimage)
            self.DrawLabels()

        # Update the status bar to show the current drone status & battery level
        self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

    def ReceiveImage(self,data):
        # Indicate that new data has been received (thus we are connected)
        self.communicationSinceTimer = True

        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()

    def ReceiveNavdata(self,navdata):
        # Indicate that new data has been received (thus we are connected)
        self.communicationSinceTimer = True

        # Update the message to be displayed
        msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

        self.tagLock.acquire()
        try:
            if navdata.tags_count > 0:
                self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
            else:
                self.tags = []
        finally:
            self.tagLock.release()

if __name__=='__main__':
    import sys
    rospy.init_node('ardrone_video_display')
    app = QtGui.QApplication(sys.argv)
    display = DroneVideoDisplay()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)

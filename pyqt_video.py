import cv2 as cv
import socket
import os
import platform
import numpy as np
import sys
from PySide6 import QtCore, QtWidgets, QtSerialPort, QtGui
from PySide6.QtCore import QThread, Qt, Signal, Slot
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel
import serial.tools.list_ports

class CommSock:
    def __init__(self):
        self.server_address = './videoframes'
    
        # Make sure the socket does not already exist
        try:
            os.unlink(self.server_address)
        except OSError:
            if os.path.exists(self.server_address):
                raise
            
        # header is currently totalbytes:width:height:type
        self.header_size = 16
        
        self.ipaddr = 'localhost'
        self.port = 4610
        
        self.cmdsock = self.open_control_socket();
        
        # Create a UDS socket
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        
        # Bind the socket to the address
        self.sock.bind(self.server_address)
        
        # Listen for incoming connections
        self.sock.listen(1)
        
        self.start_videoframes()
        
#        print('waiting for a connection')
        self.connection, self.client_address = self.sock.accept()
        
#        print("connected")
        
        def __del__(self):
            self.cmdsock.close()
            self.connection.close()
            
    # Commands for communicating with VideoStream over TCP/IP socket (port 4610)
    def open_control_socket(self):
        # Create a control socket for sending commands to VideoStream
        videostream_address = (self.ipaddr, self.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(videostream_address)
        return sock
    
    def start_videoframes(self):
        self.cmdsock.sendall(b"vstream::domainSocketSendN 0; vstream::domainSocketOpen ./videoframes\r\n")
        self.cmdsock.recv(256)
        
    def request_videoframe(self):
        self.cmdsock.sendall(b"vstream::domainSocketSendN 1\r\n")
        self.cmdsock.recv(256)
        
    # Helper function to recv n bytes or return None if EOF is hit
    def recvall(self, n):
        data = bytearray()
        while len(data) < n:
            packet = self.connection.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

class ReceiveFramesThread(QThread):
    changePixmap = Signal(QImage)

    def __init__(self, parent = None):
#        print("Initializing Thread")
        QThread.__init__(self, parent)
        self.finished = False
    
    def run(self):
        comsock = CommSock()
        scale_prop = 0.25

        while not self.finished:
            comsock.request_videoframe()
            databytes = comsock.connection.recv(comsock.header_size)
            if databytes:
                totalbytes, height, width, mat_type = np.frombuffer(databytes, dtype=np.uint32)
                if totalbytes:
                    imagebytes = comsock.recvall(totalbytes)
                    if imagebytes:
                        depth = totalbytes//(width*height)
                        image = np.frombuffer(imagebytes, dtype=np.uint8)
                        scale_to_width, scale_to_height = (int(width*scale_prop), int(height*scale_prop))
                        bytesPerLine = width * depth
                        convertToQtFormat = QImage(image, width, height, bytesPerLine, QImage.Format_BGR888)
                        p = convertToQtFormat.scaled(scale_to_width, scale_to_height, Qt.KeepAspectRatio)
                        self.changePixmap.emit(p)

class LaserControl(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(LaserControl, self).__init__(parent)
        self.title = 'Laser Control'
        self.left = 100
        self.top = 100
        self.width = 640
        self.height = 480

        # initialize the serial port at the end
        self.serial = QtSerialPort.QSerialPort(
            None, baudRate=115200, readyRead=self.receive_serial
        )
        self.ports = [comport.device for comport in serial.tools.list_ports.comports()]
        self.serial_connected = False

        self.initUI()

        self.x = 0
        self.y = 0

        self.th = ReceiveFramesThread()
        self.th.changePixmap.connect(self.setImage)
        self.th.start()
        self.show()
        
    def update_dacs(self):
        print(f"DACS: {self.x} {self.y}")
        if not self.serial_connected:
            self.serial_connect()
        else:
            self.serial.write(f'{self.x} {self.y}\n'.encode())
        
    def update_x(self, val):
        self.x = val
        self.xslider_vlabel.setText(f"{val:4}")
        self.update_dacs()

    def update_y(self, val):
        self.y = val
        self.yslider_vlabel.setText(f"{val:4}")
        self.update_dacs()
        
    @Slot(QImage)
    def setImage(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))

    def initUI(self):
        centralWidget = QtWidgets.QWidget()
        self.setCentralWidget(centralWidget)

        self.setWindowTitle(self.title)
#        self.setGeometry(self.left, self.top, self.width, self.height)
#        self.resize(1024, 768)
        # create a label

        self.connect_label = QtWidgets.QLabel("Port:")
        self.connect_path = QtWidgets.QComboBox()
        self.connect_path.currentIndexChanged.connect(self.set_connect_path)
        
        self.xslider_frame = QtWidgets.QHBoxLayout()
        self.xslider_label = QtWidgets.QLabel("X:")
        self.xslider_slider = QtWidgets.QSlider()
        self.xslider_vlabel = QtWidgets.QLabel("0")
        self.xslider_slider.setRange(0,4095)
        self.xslider_slider.valueChanged.connect(self.update_x)
        self.xslider_frame.addWidget(self.xslider_label, alignment=QtCore.Qt.AlignTop)
        self.xslider_frame.addWidget(self.xslider_slider, alignment=QtCore.Qt.AlignLeft)
        self.xslider_frame.addWidget(self.xslider_vlabel, alignment=(QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop), stretch=2)

        self.yslider_frame = QtWidgets.QHBoxLayout()
        self.yslider_label = QtWidgets.QLabel("Y:")
        self.yslider_vlabel = QtWidgets.QLabel("0")
        self.yslider_slider = QtWidgets.QSlider()
        self.yslider_slider.setRange(0,4095)
        self.yslider_slider.valueChanged.connect(self.update_y)
        self.yslider_frame.addWidget(self.yslider_label, alignment=QtCore.Qt.AlignTop)
        self.yslider_frame.addWidget(self.yslider_slider, alignment=(QtCore.Qt.AlignLeft))
        self.yslider_frame.addWidget(self.yslider_vlabel, alignment=(QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop), stretch=2)
         
        self.label = QLabel(self)
        self.label.resize(480, 270)

        row = 0

        # Setup grid to have first column 20% of width and second column 80%
        grid = QtWidgets.QGridLayout(centralWidget)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 4)
#        grid.setColumnMinimumWidth(0, 50)

        grid.addWidget(self.connect_label, row, 0, QtCore.Qt.AlignLeft)
        grid.addWidget(self.connect_path, row, 1)
        row += 1
        
        grid.addLayout(self.xslider_frame, row, 0)
        grid.addLayout(self.yslider_frame, row, 1)
        row+=1
        
        grid.addWidget(self.label, row, 0, 1, 2)

        self.refresh_ports()

    def receive_serial(self):
        while self.serial.canReadLine():
            input_line = self.serial.readLine().data().decode("utf-8")
            input_line = input_line.strip('\r\n')
            print(input_line)
        
    def refresh_ports(self):
        '''find serial ports and return default'''
        if platform.system == "Linux":
            ports = [comport.device for comport in serial.tools.list_ports.comports()]
        else:
            ports = [comport.device for comport in serial.tools.list_ports.comports()]
        self.connect_path.addItems(ports)
        if ports:
            self.connect_path.setCurrentIndex(len(ports)-1)        
        
    def set_connect_path(self):
        self.serial.setPortName(self.connect_path.currentText())
        
    def serial_connect(self):
        if not self.serial.isOpen():
            
            # On Windows we open and close the serial port using PySerial to initialize
            if platform.system() == "Windows":
                ser = serial.Serial(w.connect_path.currentText(), 9600)
                ser.close()

            self.set_connect_path()
            if not self.serial.open(QtCore.QIODeviceBase.ReadWrite):
                print("error opening")
                self.serial_connected = False
            else:
                self.serial_connected = True
        self.serial.clear()

        
        
    def closeEvent(self, event):
#        print("shutting down")
        self.th.finished = True
        while not self.th.isFinished():
            pass
        

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = LaserControl()
    w.resize(500, 500)

#    w.show()

    sys.exit(app.exec())

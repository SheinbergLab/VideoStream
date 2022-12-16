#
# NAME
#   receive_frames.py
#
# REQUIREMENTS
#   opencv (cv2) and numpy
#
# DESCRIPTION
#   Demonstration of open Unix Domain Socket to receive video frames from
#  the VideoStream program.  This program opens the socket and binds to it.
#  It then enters a receive loop, waiting for data on the socket.  The data
#  arrive as a 32 bytes header (total data, width, height, type) followed by
#  the full data stream (total data bytes).  These data are converted into a
#  numpy array and then an opencv matrix.  The matrix is rescaled and displayed. 
#
#   In contrast to receive_video.py, this program opens a control TCP/IP socket
#  with VideoStream and sends the command to start the domain socket itself.  It
#  configures VideoStream to not send any frames after opening the socket, and
#  requests these one every second.
#    
#

import socket
import os
import numpy as np
import cv2 as cv
import time

from tkinter import *
from PIL import Image, ImageTk
import tkinter as tk

class VideoFeed:
    def __init__(self, label):
        self.label = label

        self.server_address = './videoframes'
        self.scale_prop = 0.25
        self.update_interval = 8
        

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

        print('waiting for a connection')
        self.connection, self.client_address = self.sock.accept()

        print("connected")
        
        self.display_frame()

    def __del__(self):
        self.cmdsock.close()
        self.connection.close()
        self.label.after_cancel(self._job)
        
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

    def display_frame(self):
        self.request_videoframe()
        databytes = self.connection.recv(self.header_size)
        if databytes:
            totalbytes, width, height, mat_type = np.frombuffer(databytes, dtype=np.uint32)
            if totalbytes:
                imagebytes = self.recvall(totalbytes)
                if imagebytes:
                    depth = totalbytes//(width*height)
                    image = np.frombuffer(imagebytes, dtype=np.uint8).reshape(width, height, depth)
                    dim = (int(image.shape[1]*self.scale_prop), int(image.shape[0]*self.scale_prop))
                    resized = cv.resize(image, dim, interpolation=cv.INTER_AREA)                    
                    gray = cv.cvtColor(resized, cv.COLOR_BGR2GRAY)
                    
                    img = Image.fromarray(gray);
                
                    imgtk = ImageTk.PhotoImage(image = img)
                    self.label.imgtk = imgtk
                    self.label.configure(image=imgtk)

        self._job = self.label.after(self.update_interval, self.display_frame)

class LaserGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LaserGui")
        # create two variables to store position
        self.x = tk.IntVar()
        self.y = tk.IntVar()

        # create labels for x and y and put these in column 0
        self.xlabel = tk.Label(self, text="X Pos")
        self.ylabel = tk.Label(self, text="Y Pos")
        self.xlabel.grid(row=0, column=0)
        self.ylabel.grid(row=1, column=0)

        # create horizontal sliders for x and y settings and put in column 1
        #  the command= provides a callback function that will be called
        #  automatically when the slider moves
        self.xslider = tk.Scale(self, from_=0, to=4095, orient=tk.HORIZONTAL,
                           variable=self.x, command=self.update_dacs)
        self.yslider = tk.Scale(self, from_=0, to=4095, orient=tk.HORIZONTAL,
                            variable=self.y, command=self.update_dacs)
        self.xslider.grid(row=0, column=1)
        self.yslider.grid(row=1, column=1)

        # Create a Label to capture the Video frames
        label = tk.Label(self)

        self.videofeed = VideoFeed(label)
        label.grid(row=2, column=0, columnspan=2)
        
    def update_dacs(self, val):
        # change this to actually send the command to your Arduino
        #  over the serial port
        print(f"Send DAC Update {self.x.get()} {self.y.get()}")


LaserGui().mainloop()



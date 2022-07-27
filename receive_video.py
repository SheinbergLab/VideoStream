#
# NAME
#   receive_video.py
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
# USAGE
#    After starting this program, send the following command to VideoStream
#
#    vstream::domainSocketOpen ./videoframes
#     (where ./videoframes refers to the unix domain socket opened in this program)
#

import socket
import os
import numpy as np
import cv2 as cv


server_address = './videoframes'

# header is currently totalbytes:width:height:type
header_size = 16

def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


# Make sure the socket does not already exist
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        raise

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

# Bind the socket to the address
print('starting up on {}'.format(server_address))
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

scale_prop = 0.25

while True:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = sock.accept()
    try:
        print('connection from', client_address)

        # Receive the data 
        while True:
            databytes = connection.recv(header_size)
            if databytes:
                totalbytes, width, height, mat_type = np.frombuffer(databytes, dtype=np.uint32)
                if totalbytes:
                    imagebytes = recvall(connection, totalbytes)
                    if imagebytes:
                        depth = totalbytes//(width*height)
                        image = np.frombuffer(imagebytes, dtype=np.uint8).reshape(width, height, depth)
                        dim = (int(image.shape[1]*scale_prop), int(image.shape[0]*scale_prop))
                        resized = cv.resize(image, dim, interpolation=cv.INTER_AREA)                    
                        gray = cv.cvtColor(resized, cv.COLOR_BGR2GRAY)
                        cv.imshow('frame', gray)
                        cv.waitKey(1)
                    else:
                        break
            else:
                break
    finally:
        # Clean up the connection
        connection.close()

# socket_echo_server_uds.py
import socket
import sys
import os
import numpy as np

server_address = './videoframes'

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

while True:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = sock.accept()
    try:
        print('connection from', client_address)

        # Receive the data 
        while True:
            databytes = connection.recv(16)
            if databytes:
                data = np.frombuffer(databytes, dtype=np.uint32)
                print(f'received: {data}')
            else:
                break
    finally:
        # Clean up the connection
        connection.close()

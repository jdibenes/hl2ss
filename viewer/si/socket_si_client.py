# OK

import socket
import numpy as np
import struct

# hololens 2 address
HOST = "192.168.1.15"
PORT = 3812

#------------------------------------------------------------------------------

data = bytearray()
state = 0
chunk_size = 1024
packet_size = 8 + 40 + 28

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    
    while True:
        chunk = s.recv(chunk_size) 
        if (len(chunk) == 0): break
        data.extend(chunk)

        while (len(data) >= packet_size):
            packet = struct.unpack('<QIfffffffffIffffff', data[:packet_size])
            data = data[packet_size:]
            print(packet)
            
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
packet_size = 8 + 1 + 36 + 24 + 26*36 + 26*36

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    
    while True:
        chunk = s.recv(chunk_size) 
        if (len(chunk) == 0): break
        data.extend(chunk)

        while (len(data) >= packet_size):
            packet = struct.unpack('<QBfffffffffffffff', data[:(8+1+36+24)])
            data = data[packet_size:]
            print(packet) # print timestamp (Q), valid structs (B), head pose (9 floats), eye ray (6 floats)
            
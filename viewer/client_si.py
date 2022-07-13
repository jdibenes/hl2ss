# OK

import hl2ss
import socket
import numpy as np
import struct

# hololens 2 address
host = "192.168.1.15"

# Port number
port = hl2ss.StreamPort.SPATIAL_INPUT

chunk_size = 1024

#------------------------------------------------------------------------------

unpacker = hl2ss.unpacker(0)
client = hl2ss.client()

client.open(host, port)

while (client.recv(chunk_size)):
    data_available = unpacker.unpack(client.get_chunk())
    if (data_available):
        payload = unpacker.get_payload()
        packet = struct.unpack('<Bfffffffffffffff', payload[:(1+(4*15))])
        #print(unpacker.get_timestamp())
        print(packet) # print valid structs (B), head pose (9 floats), eye ray (6 floats)

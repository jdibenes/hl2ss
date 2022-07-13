
import hl2ss 
import cv2
import numpy as np

host = "192.168.1.15"
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW
mode = hl2ss.StreamMode.MODE_0
chunk_size = 4096
factor = 8
pose_frequency = 5

#--------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.get_mode2_rm_depth(host, port)

    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)

    quit()





client = hl2ss.gatherer()
frames = 0

client.open(host, port, chunk_size, mode)
client.configure(hl2ss.create_configuration_for_mode(mode))

while True:
    data = client.get_next_packet()

    images = hl2ss.unpack_rm_depth(data.payload)
    depth = images[0]
    ab = images[1]
    frames += 1

    if (mode == hl2ss.StreamMode.MODE_1 and frames >= pose_frequency):
        frames = 0
        print('Pose at {timestamp}'.format(timestamp=data.timestamp))
        print(data.pose)
        
    cv2.imshow('depth', depth*factor)
    cv2.imshow('ab', ab*factor)
    cv2.waitKey(1)

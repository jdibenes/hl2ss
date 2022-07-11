
from distutils.log import error
import socket
import cv2
import numpy as np
import select
import time
from time import sleep
import struct

HOST = "192.168.1.15"
PORT = 3805

#depth value in mm
# div by 1000 for meters
#~5fps
# ~10% CPU

start_qpc = 0
frames = 0
tmpstate = 0
pngfull = np.zeros((288, 320), dtype=np.uint16)
mode = 1

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(struct.pack('<B', mode))
    packet = bytearray()
    while True:
        chunk = s.recv(1024)
        if (len(chunk) == 0): quit() # connection closed
        packet.extend(chunk)

        if (tmpstate == 0):
            if (len(packet) >= 12):
                header = struct.unpack('<QI', packet[:12])
                timestamp = header[0]
                pngsize = header[1]
                packetlen = 12 + pngsize + 64
                tmpstate = 1
            continue
        elif (tmpstate == 1):
            if (len(packet) >= packetlen):
                szdata = packet[12:(packetlen - 64)]
                posedata = packet[(packetlen - 64):packetlen]
                packet = packet[packetlen:]
                tmpstate = 0
            else:
                continue

        composite = cv2.imdecode(np.frombuffer(szdata, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        b,g,r,a = cv2.split(composite)

        cv2.imshow('b', b)
        cv2.imshow('r', r)

        cv2.waitKey(1)

        if (start_qpc == 0):
            start_qpc = 1
            last_cap_time = time.perf_counter()
        else:
            frames += 1
            if (frames >= 10):
                print(frames/(time.perf_counter() - last_cap_time))
                last_cap_time = time.perf_counter()
                frames = 0
                pose = np.frombuffer(posedata, dtype=np.float32).reshape((4,4))
                print(pose)

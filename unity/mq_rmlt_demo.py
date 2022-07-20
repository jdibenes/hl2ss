
import socket
import cv2
import hl2ss
import rus
import keyboard
import numpy as np

host = '192.168.1.15'
port_mq = 3816
port_lt = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

with open('set_texture_test.jpg', mode='rb') as jpg:
    image = jpg.read()

ipc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ipc.connect((host, port_mq))

calib  = hl2ss.download_calibration_rm_depth(host, port_lt)
print(calib.extrinsics)
client = hl2ss.connect_client_rm_depth(host, port_lt, 4096, hl2ss.StreamMode.MODE_1)
state = False

while True:
    data = client.get_next_packet()
    images = hl2ss.unpack_rm_depth(data.payload)

    if (keyboard.is_pressed('space')):
        if (not state):
            M = np.linalg.inv(calib.extrinsics) @ data.pose
            p = np.array([hl2ss.Resolution_RM_DEPTH_LONGTHROW.WIDTH / 2, hl2ss.Resolution_RM_DEPTH_LONGTHROW.HEIGHT / 2]).reshape((1, 2))
            u0 = int(p[0,0])
            v0 = int(p[0,1])

            points = np.array([]).reshape((0, 4))
            for dv in range(-3, 4):
                for du in range(-3, 4):
                    v = v0 + dv
                    u = u0 + du
                    xy = calib.uv2xy[v][u]
                    xy1 = np.hstack((xy, 1)) # normalize?
                    z = images.depth[v][u] / calib.scale
                    if (z == 0):
                        continue
                    xyz1 = np.hstack((xy1*z, 1))
                    xyz1_world = np.matmul(xyz1, M)
                    points = np.vstack((points, xyz1_world))

            centroid = np.mean(points, axis=0)
            xu,xs,xvh = np.linalg.svd(points)
            normal = xvh[3,0:3]
            normal = normal / np.linalg.norm(normal)

            direction = -centroid[0:3]
            if (np.dot(normal, direction) < np.dot(-normal, direction)):
                normal = -normal

            #convert to left handed ?
            normal[2] = -normal[2]
            start = np.array([0,0,-1]).reshape((1, 3))
            axis = np.cross(start, normal)
            axis = axis/np.linalg.norm(axis)
            dotp = np.dot(start, normal)
            print(dotp)
            if (dotp > 1):
                dotp = 1
            elif (dotp < -1):
                dotp = -1
            angle = np.arccos(dotp)
            cos = np.cos(angle/2)
            sin = np.sin(angle/2)
            rx = axis[0,0] * sin
            ry = axis[0,1] * sin
            rz = axis[0,2] * sin
            rw = cos

            rus.create_primitive(ipc, rus.PrimitiveType.Quad)
            key = rus.get_result(ipc)
            print(key)

            #left handed coordinate system
            rus.set_transform(ipc, key, 0, centroid[0], centroid[1], -centroid[2], rx, ry, rz, rw, 0.2, 0.2, 1)
            ret = rus.get_result(ipc)
            print(ret)

            rus.set_texture(ipc, key, image)
            ret = rus.get_result(ipc)
            print(ret)

        state = True
    else:
        state = False

    cv2.imshow('depth', images.depth*8)
    cv2.waitKey(1)

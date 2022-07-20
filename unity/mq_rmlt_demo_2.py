
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

with open('set_texture_test_2.jpg', mode='rb') as jpg:
    image_1 = jpg.read()

image_id = 0

ipc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ipc.connect((host, port_mq))

rus.remove_all(ipc)
rus.get_results(ipc, 1)

calib  = hl2ss.download_calibration_rm_depth(host, port_lt)
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
                    xy1 = xy1 / np.linalg.norm(xy1)
                    z = images.depth[v][u] / calib.scale
                    if (z == 0):
                        continue
                    xyz1 = np.hstack((xy1*z, 1))
                    xyz1_world = np.matmul(xyz1, M)
                    points = np.vstack((points, xyz1_world))

            if (points.shape[0] < 3):
                print('not enough points')
                continue

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


            key = 0
            if (image_id == 0):
                tex=image
                r = 1.0
                g = 0.0
                b = 1.0
                a = 1.0
            else:
                tex=image_1
                r = 0.0
                g = 1.0
                b = 0.0
                a = 1.0
            image_id = image_id ^ 1

            rus.begin_display_list(ipc) # Begin sequence
            rus.create_primitive(ipc, rus.PrimitiveType.Quad) # Create quad, returns id which can be used to modify its properties
            rus.set_target_mode(ipc, 1) # Set server to use the last created object as target (this avoids waiting for the id)
            rus.set_world_transform(ipc, key, centroid[0], centroid[1], -centroid[2], rx, ry, rz, rw, 0.2, 0.2, 1) # Set the quad's world transform
            rus.set_texture(ipc, key, tex) # Set the quad's texture
            rus.set_active(ipc, key, 1) # Make the quad visible
            rus.set_target_mode(ipc, 0) # Restore target mode
            rus.end_display_list(ipc) # End sequence

            results = rus.get_results(ipc, 8)
            key = results[1]
            print('Created quad with id {iid}'.format(iid=key))
            print(results)


        state = True
    else:
        state = False

    cv2.imshow('depth', images.depth*8)
    cv2.waitKey(1)


            #rus.create_text(ipc)
            #rus.set_text(ipc, key, 0.2, r, g, b, a, "HELLO")
            #rus.set_world_transform(ipc, key, centroid[0], centroid[1], -centroid[2], rx, ry, rz, rw, 1, 1, 1)
            #rus.set_local_transform(ipc, key, int(1440/2), int(936/2), 40, 0, 0, 0, 1, 256, 256, 1)
            #rus.set_color(ipc, key, r, g, b, a)
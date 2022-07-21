
import cv2
import hl2ss
import rus
import keyboard
import numpy as np

host    = '192.168.1.15'
port_mq = rus.IPCPort.MESSAGE_QUEUE
port_lt = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

def clamp(v, min, max):
    if (v < min):
        return min
    elif (v > max):
        return max
    else:
        return v


# Load texture
with open('set_texture_test.jpg', mode='rb') as jpg:
    image = jpg.read()

# Connect to HoloLens
ipc = rus.connect_client_mq(host, port_mq)

command_buffer = bytearray()
rus.remove_all(command_buffer)
ipc.sendall(command_buffer)

results = rus.get_results(ipc, 1)

state  = False
key    = 0
calib  = hl2ss.download_calibration_rm_depth(host, port_lt)
client = hl2ss.connect_client_rm_depth(host, port_lt, 4096, hl2ss.StreamMode.MODE_1)

while True:
    # Wait for next depth image
    data = client.get_next_packet()
    images = hl2ss.unpack_rm_depth(data.payload)

    if (keyboard.is_pressed('space')):
        if (not state):
            # 4x4 matrix that converts 3D points in depth camera space to world space
            camera2world = np.linalg.inv(calib.extrinsics) @ data.pose

            # Get the 3D points corresponding to the 7x7 patch in the center of the depth image
            u0 = int(hl2ss.Resolution_RM_DEPTH_LONGTHROW.WIDTH  / 2)
            v0 = int(hl2ss.Resolution_RM_DEPTH_LONGTHROW.HEIGHT / 2)

            points = np.array([]).reshape((0, 4))
            for dv in range(-3, 4):
                for du in range(-3, 4):
                    v = v0 + dv
                    u = u0 + du

                    z = images.depth[v][u] / calib.scale
                    if (z == 0):
                        continue

                    xy1 = np.hstack((calib.uv2xy[v][u], 1))
                    xy1 = xy1 / np.linalg.norm(xy1)
                    points = np.vstack((points, np.matmul(np.hstack((xy1*z, 1)), camera2world)))

            # Need at least 3 points to fit a plane
            if (points.shape[0] < 3):
                print('Not enough points')
                continue

            # Compute plane
            _, _, vh = np.linalg.svd(points)               
            plane = vh[3, :]
            plane = plane / np.linalg.norm(plane[0:3])
            normal = plane[0:3]

            # Compute centroid
            centroid = np.mean(points, axis=0)
            centroid = centroid[0:3]
           
            # Select the normal that points to the user
            camera_center = np.matmul(np.array([0,0,0,1]).reshape((1, 4)), camera2world)
            camera_center = camera_center[0,0:4]
            direction = camera_center[0:3] - centroid
            if (np.dot(normal, direction) < np.dot(-normal, direction)):
                normal = -normal

            # Convert to left handed coordinates (Unity)
            normal[2] = -normal[2]
            centroid[2] = -centroid[2]

            # Find the axis and the angle of the rotation between the canonical normal and the plane normal
            canonical_normal = np.array([0,0,-1]).reshape((1, 3)) # Normal that looks at the camera when the camera transform is the identity
            axis = np.cross(canonical_normal, normal)
            axis = axis / np.linalg.norm(axis)
            angle = np.arccos(clamp(np.dot(canonical_normal, normal), -1, 1))

            # Quad rotation: convert axis-angle rotation to quaternion
            cos = np.cos(angle/2)
            sin = np.sin(angle/2)

            rotation = [axis[0,0] * sin, axis[0,1] * sin, axis[0,2] * sin, cos]

            # Quad scale
            scale = [0.2, 0.2, 1]

            # Add quad to Unity scene
            display_list = bytearray()
            rus.begin_display_list(display_list) # Begin sequence
            rus.create_primitive(display_list, rus.PrimitiveType.Quad) # Create quad, returns id which can be used to modify its properties
            rus.set_target_mode(display_list, 1) # Set server to use the last created object as target (this avoids waiting for the id)
            rus.set_world_transform(display_list, key, centroid, rotation, scale) # Set the quad's world transform
            rus.set_texture(display_list, key, image) # Set the quad's texture
            rus.set_active(display_list, key, 1) # Make the quad visible
            rus.set_target_mode(display_list, 0) # Restore target mode
            rus.end_display_list(display_list) # End sequence
            ipc.sendall(display_list)

            # Get the results for the display list
            results = rus.get_results(ipc, 8)
            key = results[1]
            print('Created quad with id {iid}'.format(iid=key))

        state = True
    else:
        state = False

    if (keyboard.is_pressed('esc')):
        break

    # Show depth image
    cv2.imshow('depth', images.depth*8)
    cv2.waitKey(1)


command_buffer = bytearray()
rus.remove_all(command_buffer)
ipc.sendall(command_buffer)

results = rus.get_results(ipc, 1)

ipc.close()

#------------------------------------------------------------------------------
# This script uses depth frames received from the HoloLens to place textured
# quads in the Unity scene aligned with the surface the user is observing.
# A patch of 3D points at the center of the depth frame is used to fit a plane,
# which is then used to set the world transform of the quad such that the quad
# is aligned with observed surface (e.g., a wall).
#------------------------------------------------------------------------------

import cv2
import hl2ss
import rus
import keyboard
import numpy as np

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Ports
port_mq = rus.Port.IPC
port_lt = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Quad scale in meters
scale = [0.2, 0.2, 1]

# Texture file (must be jpg or png)
texture_file = 'texture.jpg'

# Place a quad when this key is pressed
trigger = 'space'

#------------------------------------------------------------------------------

def clamp(v, min, max):
    if (v < min):
        return min
    elif (v > max):
        return max
    else:
        return v

with open(texture_file, mode='rb') as file:
    image = file.read()

ipc = rus.connect_client_mq(host, port_mq)
key = 0

command_buffer = rus.create_command_buffer()
command_buffer.remove_all()
ipc.push(command_buffer)
results = ipc.pop(command_buffer)

state  = False
calib  = hl2ss.download_calibration_rm_depth(host, port_lt)
client = hl2ss.connect_client_rm_depth(host, port_lt, 4096, hl2ss.StreamMode.MODE_1)

try:
    while True:
        data = client.get_next_packet()
        images = hl2ss.unpack_rm_depth(data.payload)

        if (keyboard.is_pressed(trigger)):
            if (not state):
                # 4x4 matrix that converts 3D points in depth camera space to world space
                camera2world = np.linalg.inv(calib.extrinsics) @ data.pose

                # Get the 3D points corresponding to the 7x7 patch in the center of the depth image
                u0 = int(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH  / 2)
                v0 = int(hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT / 2)

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

                # Fit plane
                _, _, vh = np.linalg.svd(points)               
                plane = vh[3, :]
                plane = plane / np.linalg.norm(plane[0:3])
                normal = plane[0:3]

                # Compute centroid
                centroid = np.median(points, axis=0)
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

                # Convert axis-angle rotation to quaternion
                cos = np.cos(angle/2)
                sin = np.sin(angle/2)

                rotation = [axis[0,0] * sin, axis[0,1] * sin, axis[0,2] * sin, cos]

                # Add quad to Unity scene
                display_list = rus.create_command_buffer()
                display_list.begin_display_list() # Begin sequence
                display_list.create_primitive(rus.PrimitiveType.Quad) # Create quad, returns id which can be used to modify its properties
                display_list.set_target_mode(1) # Set server to use the last created object as target (this avoids waiting for the id)
                display_list.set_world_transform(key, centroid, rotation, scale) # Set the quad's world transform
                display_list.set_texture(key, image) # Set the quad's texture
                display_list.set_active(key, 1) # Make the quad visible
                display_list.set_target_mode(0) # Restore target mode
                display_list.end_display_list() # End sequence
                ipc.push(display_list) # Send commands to server
                results = ipc.pop(display_list) # Get results from server
                key = results[1]

                print('Created quad with id {iid}'.format(iid=key))

            state = True
        else:
            state = False

        # Show depth image
        cv2.imshow('depth', images.depth*8)
        cv2.waitKey(1)
except:
    pass

command_buffer = rus.create_command_buffer()
command_buffer.remove_all()
ipc.push(command_buffer)
results = ipc.pop(command_buffer)

ipc.close()

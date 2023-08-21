#------------------------------------------------------------------------------
# This script uses depth frames received from the HoloLens to place textured
# quads in the Unity scene aligned with the surface the user is observing.
# A patch of 3D points at the center of the depth frame is used to fit a plane,
# which is then used to set the world transform of the quad such that the quad
# is aligned with observed surface (e.g., a wall).
# Press space to set a quad.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_rus

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# Quad scale in meters
scale = [0.2, 0.2, 1]

# Texture file (must be jpg or png)
texture_file = 'texture.jpg'

# Scaling factor for visibility
brightness = 8

#------------------------------------------------------------------------------

enable = True
trigger = False

def clamp(v, min, max):
    return min if (v < min) else max if (v > max) else v

def on_press(key):
    global enable
    global trigger
    if (key == keyboard.Key.esc):
        enable = False
    elif (key == keyboard.Key.space):
        trigger = True
    enable = key != keyboard.Key.esc
    return enable

def on_release(key):
    global trigger
    if (key == keyboard.Key.space):
        trigger = False
    return True

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

with open(texture_file, mode='rb') as file:
    image = file.read()

ipc = hl2ss_lnm.ipc_umq(host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE)
ipc.open()

key = 0

command_buffer = hl2ss_rus.command_buffer()
command_buffer.remove_all()
ipc.push(command_buffer)
results = ipc.pull(command_buffer)

previous  = False

calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
xy1, lt_scale = hl2ss_3dcv.rm_depth_compute_rays(calibration_lt.uv2xy, calibration_lt.scale)

u0 = hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH  // 2
v0 = hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT // 2

client = hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
client.open()

while (enable):
    data = client.get_next_packet()
    data_lt = data.payload

    # Show depth image
    cv2.imshow('depth', data_lt.depth / np.max(data_lt.depth)) # Scaled for visibility
    cv2.waitKey(1)

    keydown = (not previous) and trigger
    previous = trigger

    if ((not hl2ss.is_valid_pose(data.pose)) or (not keydown)):
        continue

    # Get the 3D points corresponding to the 7x7 patch in the center of the depth image
    depth = hl2ss_3dcv.rm_depth_normalize(data_lt.depth[(v0-3):(v0+4), (u0-3):(u0+4)], lt_scale[(v0-3):(v0+4), (u0-3):(u0+4)])
    xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1[(v0-3):(v0+4), (u0-3):(u0+4), :])
    xyz = hl2ss_3dcv.block_to_list(xyz)
    d = hl2ss_3dcv.block_to_list(depth).reshape((-1,))
    xyz = xyz[d > 0, :]

    # Need at least 3 points to fit a plane
    if (xyz.shape[0] < 3):
        print('Not enough points')
        continue

    # 4x4 matrix that converts 3D points in depth camera space to world space
    camera2world = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data.pose)
    points = hl2ss_3dcv.to_homogeneous(xyz) @ camera2world

    # Fit plane
    _, _, vh = np.linalg.svd(points)
    plane = vh[3, :]
    plane = plane / np.linalg.norm(plane[0:3])
    normal = plane[0:3]

    # Compute centroid
    centroid = np.median(points, axis=0)
    centroid = centroid[0:3]
        
    # Select the normal that points to the user
    camera_center = np.array([0, 0, 0, 1]).reshape((1, 4)) @ camera2world
    camera_center = camera_center[0, 0:4]
    direction = camera_center[0:3] - centroid
    if (np.dot(normal, direction) < np.dot(-normal, direction)):
        normal = -normal

    # Convert to left handed coordinates (Unity)
    normal[2] = -normal[2]
    centroid[2] = -centroid[2]

    # Find the axis and the angle of the rotation between the canonical normal and the plane normal
    canonical_normal = np.array([0, 0, -1]).reshape((1, 3)) # Normal that looks at the camera when the camera transform is the identity
    axis = np.cross(canonical_normal, normal)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(clamp(np.dot(canonical_normal, normal), -1, 1))

    # Convert axis-angle rotation to quaternion
    cos = np.cos(angle / 2)
    sin = np.sin(angle / 2)

    rotation = [axis[0,0] * sin, axis[0,1] * sin, axis[0,2] * sin, cos]

    # Add quad to Unity scene
    display_list = hl2ss_rus.command_buffer()
    display_list.begin_display_list() # Begin sequence
    display_list.create_primitive(hl2ss_rus.PrimitiveType.Quad) # Create quad, returns id which can be used to modify its properties
    display_list.set_target_mode(1) # Set server to use the last created object as target (this avoids waiting for the id)
    display_list.set_world_transform(key, centroid, rotation, scale) # Set the quad's world transform
    display_list.set_texture(key, image) # Set the quad's texture
    display_list.set_active(key, 1) # Make the quad visible
    display_list.set_target_mode(0) # Restore target mode
    display_list.end_display_list() # End sequence

    ipc.push(display_list) # Send commands to server
    results = ipc.pull(display_list) # Get results from server
    
    key = results[1]

    print(f'Created quad with id {key}')

command_buffer = hl2ss_rus.command_buffer()
command_buffer.remove_all()

ipc.push(command_buffer)
results = ipc.pull(command_buffer)

ipc.close()

listener.join()

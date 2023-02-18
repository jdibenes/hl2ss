#------------------------------------------------------------------------------
# This script uses depth frames received from the HoloLens to place textured
# quads in the Unity scene aligned with the surface the user is observing.
# A patch of 3D points at the center of the depth frame is used to fit a plane,
# which is then used to set the world transform of the quad such that the quad
# is aligned with observed surface (e.g., a wall). Press space to set a quad,
# esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_3dcv
import hl2ss_utilities
import rus
import numpy as np

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Ports
port_mq = rus.Port.IPC
port_lt = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Quad scale in meters
scale = [0.2, 0.2, 1]

# Texture file (must be jpg or png)
texture_file = 'texture.jpg'

# Scaling factor for visibility
brightness = 8

#------------------------------------------------------------------------------

enable = True
trigger = False

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

ipc = rus.connect_client_mq(host, port_mq)
key = 0

command_buffer = rus.create_command_buffer()
command_buffer.remove_all()
ipc.push(command_buffer)
results = ipc.pop(command_buffer)

previous  = False
calib  = hl2ss.download_calibration_rm_depth_longthrow(host, port_lt)
scaler = np.sqrt(calib.uv2xy[:, :, 0]**2 + calib.uv2xy[:, :, 1]**2 + 1) * calib.scale

u0 = hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH  // 2
v0 = hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT // 2

client = hl2ss.rx_decoded_rm_depth_longthrow(host, port_lt, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, hl2ss.PngFilterMode.Paeth)
client.open()

while (enable):
    data = client.get_next_packet()
    #images = hl2ss.unpack_rm_depth(data.payload)
    images = data.payload

    # Show depth image
    cv2.imshow('depth', images.depth / np.max(images.depth))
    cv2.waitKey(1)

    keydown = (not previous) and trigger
    previous = trigger

    if ((not hl2ss.is_valid_pose(data.pose)) or (not keydown)):
        continue

    # Get the 3D points corresponding to the 7x7 patch in the center of the depth image
    z = images.depth[(v0-3):(v0+4), (u0-3):(u0+4)] / scaler[(v0-3):(v0+4), (u0-3):(u0+4)]
    xy = calib.uv2xy[(v0-3):(v0+4), (u0-3):(u0+4), :] * z.reshape((z.shape[0], z.shape[1], 1))
    xyz = np.hstack((xy[:, :, 0].reshape((-1, 1)), xy[:, :, 1].reshape((-1, 1)), z.reshape((-1, 1))))
    xyz = xyz[xyz[:, 2] > 0, :]

    # Need at least 3 points to fit a plane
    if (xyz.shape[0] < 3):
        print('Not enough points')
        continue

    # 4x4 matrix that converts 3D points in depth camera space to world space
    camera2world = hl2ss_3dcv.camera_to_rignode(calib.extrinsics) @ hl2ss_3dcv.reference_to_world(data.pose)
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
    angle = np.arccos(hl2ss_utilities.clamp(np.dot(canonical_normal, normal), -1, 1))

    # Convert axis-angle rotation to quaternion
    cos = np.cos(angle / 2)
    sin = np.sin(angle / 2)

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


command_buffer = rus.create_command_buffer()
command_buffer.remove_all()

ipc.push(command_buffer)
results = ipc.pop(command_buffer)

ipc.close()

listener.join()

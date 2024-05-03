#------------------------------------------------------------------------------
# Experimental ArUco detection demo.
# Displays a holographic sphere on top of the first detected ArUco marker on
# PV video.
# Run the Unity sample on the HoloLens and then run this script.
# Press Esc to stop.
# Based on https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
#------------------------------------------------------------------------------

from pynput import keyboard
from cv2 import aruco

import numpy as np
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_rus

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Camera parameters
width_pv = 760
height_pv = 428
framerate_pv = 30

# Aruco parameters
marker_length = 0.053 # in meters

# Hologram parameters
sphere_diameter = marker_length # in meters
texture_file = './texture.jpg'

#------------------------------------------------------------------------------

# Initialize aruco detector ---------------------------------------------------
aruco_dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_parameters = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
aruco_half = marker_length / 2
aruco_reference = np.array([[-aruco_half, aruco_half, 0], [aruco_half, aruco_half, 0], [aruco_half, -aruco_half, 0], [-aruco_half, -aruco_half, 0], [0, 0, sphere_diameter / 2]], dtype=np.float32)

# Keyboard events -------------------------------------------------------------
enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

# Start PV Subsystem ----------------------------------------------------------
hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

# Connect to Unity message queue ----------------------------------------------
ipc_unity = hl2ss_lnm.ipc_umq(host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE)
ipc_unity.open()

# Create textured sphere in Unity scene ---------------------------------------
with open(texture_file, mode='rb') as file:
    texture_data = file.read()

cb_unity = hl2ss_rus.command_buffer()
cb_unity.remove_all()
cb_unity.set_target_mode(hl2ss_rus.TargetMode.UseLast)
cb_unity.create_primitive(hl2ss_rus.PrimitiveType.Sphere)
cb_unity.set_texture(0, texture_data)

sphere_scale = [sphere_diameter, sphere_diameter, sphere_diameter]

ipc_unity.push(cb_unity)
ipc_unity.pull(cb_unity)

# Start PV stream -------------------------------------------------------------
rx_pv = hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=width_pv, height=height_pv, framerate=framerate_pv)
rx_pv.open()

# Main Loop -------------------------------------------------------------------
while (enable):
    # Get PV data
    data_pv = rx_pv.get_next_packet()
    image = data_pv.payload.image

    # Update camera parameters (due to PV autofocus)
    intrinsics_pv = hl2ss.create_pv_intrinsics(data_pv.payload.focal_length, data_pv.payload.principal_point)
    extrinsics_pv = np.eye(4, 4, dtype=np.float32)
    intrinsics_pv, extrinsics_pv = hl2ss_3dcv.pv_fix_calibration(intrinsics_pv, extrinsics_pv)

    # Detect aruco marker
    corners, ids, _ = aruco_detector.detectMarkers(image)
    update_sphere = False

    if (corners and hl2ss.is_valid_pose(data_pv.pose)):
        # Estimate aruco pose
        _, aruco_rvec, aruco_tvec = cv2.solvePnP(aruco_reference[:4, :], corners[0], intrinsics_pv[:3, :3].transpose(), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        aruco_R, _ = cv2.Rodrigues(aruco_rvec)
        aruco_pose = np.eye(4, 4, dtype=np.float32)
        aruco_pose[:3, :3] = aruco_R.transpose()
        aruco_pose[3, :3] = aruco_tvec.transpose()

        # Transform aruco corners to world coordinates
        aruco_to_world = aruco_pose @ hl2ss_3dcv.camera_to_rignode(extrinsics_pv) @ hl2ss_3dcv.reference_to_world(data_pv.pose)
        aruco_reference_world = hl2ss_3dcv.transform(aruco_reference, aruco_to_world)

        # Compute sphere position and rotation in Unity scene
        sphere_position = aruco_reference_world[4, :]

        sphere_rvec, _ = cv2.Rodrigues(aruco_to_world[:3, :3])
        sphere_angle = np.linalg.norm(sphere_rvec)
        sphere_axis = sphere_rvec / sphere_angle
        sphere_quaternion = np.vstack((sphere_axis * np.sin(sphere_angle / 2), np.array([[np.cos(sphere_angle / 2)]])))[:, 0]

        sphere_position[2] = -sphere_position[2]         # right hand to left hand 
        sphere_quaternion[2:3] = -sphere_quaternion[2:3] # coordinates conversion for Unity

        update_sphere = True

    # Update sphere state
    cb_unity = hl2ss_rus.command_buffer()
    if (update_sphere):
        cb_unity.set_world_transform(0, sphere_position, sphere_quaternion, sphere_scale)
        cb_unity.set_active(0, hl2ss_rus.ActiveState.Active)
    else:
        cb_unity.set_active(0, hl2ss_rus.ActiveState.Inactive)

    ipc_unity.push(cb_unity)

    # Visualize detected markers
    image = aruco.drawDetectedMarkers(image, corners, ids)
    cv2.imshow('video', image)
    cv2.waitKey(1)


# Cleanup ---------------------------------------------------------------------
rx_pv.close()

cb_unity = hl2ss_rus.command_buffer()
cb_unity.remove_all()

ipc_unity.push(cb_unity)
ipc_unity.close()

hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

listener.join()

#------------------------------------------------------------------------------
# C++ Multithreaded Client Python Extension Test.
# Requires building the hl2ss_ulm_stream extension.
#
# This script receives spatial input data from the HoloLens, which comprises:
# 1) Head pose, 2) Eye ray, 3) Hand tracking, and prints it. 30 Hz sample rate.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import time
import hl2ss
import hl2ss_lnm
import hl2ss_mt

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Buffer size (packets)
buffer_size = 150

#------------------------------------------------------------------------------

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

configuration = hl2ss_mt.create_configuration(hl2ss.StreamPort.SPATIAL_INPUT)

client = hl2ss_mt.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT, buffer_size, configuration)
client.open()

frame_index = -1

while (enable):
    data = client.get_by_index(frame_index)
    if (data.status == hl2ss_mt.Status.WAIT):
        time.sleep(1/1000)
        continue
    elif (data.status == hl2ss_mt.Status.DISCARDED):
        frame_index = -1
        continue

    frame_index = data.frame_stamp + 1

    si = hl2ss.unpack_si(data.payload)

    print(f'Tracking status at time {data.timestamp}')

    if (si.is_valid_head_pose()):
        head_pose = si.get_head_pose()
        print(f'Head pose: Position={head_pose.position} Forward={head_pose.forward} Up={head_pose.up}')
        # right = cross(up, -forward)
        # up => y, forward => -z, right => x
    else:
        print('No head pose data')

    if (si.is_valid_eye_ray()):
        eye_ray = si.get_eye_ray()
        print(f'Eye ray: Origin={eye_ray.origin} Direction={eye_ray.direction}')
    else:
        print('No eye tracking data')

    # See
    # https://learn.microsoft.com/en-us/uwp/api/windows.perception.people.jointpose?view=winrt-22621
    # for hand data details

    if (si.is_valid_hand_left()):
        hand_left = si.get_hand_left()
        pose = hand_left.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
        print(f'Left wrist pose: Position={pose.position} Orientation={pose.orientation} Radius={pose.radius} Accuracy={pose.accuracy}')
    else:
        print('No left hand data')

    if (si.is_valid_hand_right()):
        hand_right = si.get_hand_right()
        pose = hand_right.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
        print(f'Right wrist pose: Position={pose.position} Orientation={pose.orientation} Radius={pose.radius} Accuracy={pose.accuracy}')
    else:
        print('No right hand data')

client.close()
listener.join()

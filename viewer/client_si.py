#------------------------------------------------------------------------------
# This script receives spatial input data from the HoloLens, which comprises:
# 1) Head pose, 2) Eye ray, 3) Hand tracking, and prints it. 30 Hz sample rate.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss

import zenoh
import logging

log = logging.getLogger(__name__)

# Settings --------------------------------------------------------------------

DEFAULT_KEY = "tcn/loc/hl2/*"
# most simple zenoh config for now
conf = {"mode": "peer", "queries_default_timeout": 10000}

#------------------------------------------------------------------------------

enable = True
logging.basicConfig(level=logging.DEBUG)


def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable


listener = keyboard.Listener(on_press=on_press)
listener.start()

zenoh.init_logger()

mgr = hl2ss.mgr_rpc_interface(conf, DEFAULT_KEY)

client = hl2ss.rx_si("SI", conf, DEFAULT_KEY)
client.open()

while enable:
    data = client.get_next_packet()
    if data is None:
        log.warning("Invalid packet")
        continue
    si = hl2ss.unpack_si(data.payload)

    print(f'Tracking status at time {data.timestamp}')

    if si.is_valid_head_pose():
        head_pose = si.get_head_pose()
        print(f'Head pose: Position={head_pose.position} Forward={head_pose.forward} Up={head_pose.up}')
        # right = cross(up, -forward)
        # up => y, forward => -z, right => x
    else:
        print('No head pose data')

    if si.is_valid_eye_ray():
        eye_ray = si.get_eye_ray()
        print(f'Eye ray: Origin={eye_ray.origin} Direction={eye_ray.direction}')
    else:
        print('No eye tracking data')

    # See
    # https://learn.microsoft.com/en-us/uwp/api/windows.perception.people.jointpose?view=winrt-22621
    # for hand data details

    if si.is_valid_hand_left():
        hand_left = si.get_hand_left()
        pose = hand_left.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
        print(f'Left wrist pose: Position={pose.position} Orientation={pose.orientation} Radius={pose.radius} Accuracy={pose.accuracy}')
    else:
        print('No left hand data')

    if si.is_valid_hand_right():
        hand_right = si.get_hand_right()
        pose = hand_right.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
        print(f'Right wrist pose: Position={pose.position} Orientation={pose.orientation} Radius={pose.radius} Accuracy={pose.accuracy}')
    else:
        print('No right hand data')

client.close()
listener.join()

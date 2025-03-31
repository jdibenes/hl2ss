#------------------------------------------------------------------------------
# This script receives spatial input data from the HoloLens, which comprises:
# 1) Head pose, 2) Eye ray, 3) Hand tracking, and prints it. 30 Hz sample rate.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Hand Joint to print data for
hand_joint = hl2ss.SI_HandJointKind.Wrist

#------------------------------------------------------------------------------

listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

hand_join_name = hl2ss.si_get_joint_name(hand_joint)

client = hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT)
client.open()

while (not listener.pressed()):
    data = client.get_next_packet()

    si = data.payload

    print(f'Tracking status at time {data.timestamp}')

    if (si.head_pose_valid):
        head_pose = si.head_pose
        print(f'Head pose: Position={head_pose.position} Forward={head_pose.forward} Up={head_pose.up}')
        # right = cross(up, -forward)
        # up => y, forward => -z, right => x
    else:
        print('No head pose data')

    if (si.eye_ray_valid):
        eye_ray = si.eye_ray
        print(f'Eye ray: Origin={eye_ray.origin} Direction={eye_ray.direction}')
    else:
        print('No eye tracking data')

    # See
    # https://learn.microsoft.com/en-us/uwp/api/windows.perception.people.jointpose?view=winrt-22621
    # for hand data details

    if (si.hand_left_valid):
        hand = si.hand_left
        joint_orientation = hand.orientation[hand_joint, :]
        joint_position    = hand.position[hand_joint, :]
        joint_radius      = hand.radius[hand_joint]
        joint_accuracy    = hand.accuracy[hand_joint]
        print(f'Left {hand_join_name} pose: Position={joint_position} Orientation={joint_orientation} Radius={joint_radius} Accuracy={joint_accuracy}')
    else:
        print('No left hand data')

    if (si.hand_right_valid):
        hand = si.hand_right
        joint_orientation = hand.orientation[hand_joint, :]
        joint_position    = hand.position[hand_joint, :]
        joint_radius      = hand.radius[hand_joint]
        joint_accuracy    = hand.accuracy[hand_joint]
        print(f'Right {hand_join_name} pose: Position={joint_position} Orientation={joint_orientation} Radius={joint_radius} Accuracy={joint_accuracy}')
    else:
        print('No right hand data')

client.close()
listener.close()

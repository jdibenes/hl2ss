#------------------------------------------------------------------------------
# This script receives spatial input data from the HoloLens, which comprises:
# 1) Head pose, 2) Eye ray, 3) Hand tracking. The HoloLens sends this data at
# 60 Hz.
#------------------------------------------------------------------------------

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port
port = hl2ss.StreamPort.SPATIAL_INPUT

# Print period
# Print data every 'print_period' frames (60 = 1 second)
print_period = 120

#------------------------------------------------------------------------------

frames = 0
client = hl2ss.connect_client_si(host, port, 1024)

try:
    while (True):
        data = client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)

        frames += 1
        if (frames < print_period):
            continue
        frames = 0

        print('Tracking status at time {ts}'.format(ts=data.timestamp))

        if (si.is_valid_head_pose()):
            head_pose = si.get_head_pose()
            print('Head pose')
            print(head_pose.position)
            print(head_pose.forward)
            print(head_pose.up)
            # right = cross(up, -forward)
            # up => y, forward => -z, right => x
        else:
            print('No head pose data')

        if (si.is_valid_eye_ray()):
            eye_ray = si.get_eye_ray()
            print('Eye ray')
            print(eye_ray.origin)
            print(eye_ray.direction)
        else:
            print('No eye tracking data')

        if (si.is_valid_hand_left()):
            hand_left = si.get_hand_left()
            pose = hand_left.get_joint_pose(hl2ss.HandJointKind.Wrist)
            print('Left Wrist pose')
            print(pose.orientation)
            print(pose.position)
            print(pose.radius)
            print(pose.accuracy)
        else:
            print('No left hand data')

        if (si.is_valid_hand_right()):
            hand_right = si.get_hand_right()
            pose = hand_right.get_joint_pose(hl2ss.HandJointKind.IndexTip)
            print('Right IndexTip pose')
            print(pose.orientation)
            print(pose.position)
            print(pose.radius)
            print(pose.accuracy)
        else:
            print('No right hand data')
except:
    pass

client.close()

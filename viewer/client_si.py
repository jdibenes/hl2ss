#------------------------------------------------------------------------------
# This script receives spatial input data from the HoloLens, which comprises:
# 1) Head pose, 2) Eye ray, 3) Hand tracking. Sample rate is 30 Hz.
#------------------------------------------------------------------------------

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss.StreamPort.SPATIAL_INPUT

#------------------------------------------------------------------------------

client = hl2ss.rx_si(host, port, hl2ss.ChunkSize.SPATIAL_INPUT)
client.open()

data = client.get_next_packet()
si = hl2ss.unpack_si(data.payload)

print(f'Tracking status at time {data.timestamp}')

if (si.is_valid_head_pose()):
    head_pose = si.get_head_pose()
    print('Head pose')
    print(f'Position: {head_pose.position}')
    print(f'Forward: {head_pose.forward}')
    print(f'Up: {head_pose.up}')
    # right = cross(up, -forward)
    # up => y, forward => -z, right => x
else:
    print('No head pose data')

if (si.is_valid_eye_ray()):
    eye_ray = si.get_eye_ray()
    print('Eye ray')
    print(f'Origin: {eye_ray.origin}')
    print(f'Direction: {eye_ray.direction}')
else:
    print('No eye tracking data')

# See
# https://learn.microsoft.com/en-us/uwp/api/windows.perception.people.jointpose?view=winrt-22621
# for hand data details

if (si.is_valid_hand_left()):
    hand_left = si.get_hand_left()
    pose = hand_left.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
    print('Left wrist pose')
    print(f'Orientation: {pose.orientation}') # Quaternion: x, y, z, w
    print(f'Position: {pose.position}')
    print(f'Radius: {pose.radius}')
    print(f'Accuracy: {pose.accuracy}') # 0: High, 1: Approximate
else:
    print('No left hand data')

if (si.is_valid_hand_right()):
    hand_right = si.get_hand_right()
    pose = hand_right.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
    print('Right wrist pose')
    print(f'Orientation: {pose.orientation}')  # Quaternion: x, y, z, w
    print(f'Position: {pose.position}')
    print(f'Radius: {pose.radius}')
    print(f'Accuracy: {pose.accuracy}')  # 0: High, 1: Approximate
else:
    print('No right hand data')

client.close()

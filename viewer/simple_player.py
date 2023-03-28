
import numpy as np
import pyaudio
import cv2
import os
import hl2ss
import hl2ss_io
import hl2ss_utilities

path = './data'

# Ports
port = hl2ss.StreamPort.RM_VLC_RIGHTFRONT
'''
hl2ss.StreamPort.RM_VLC_LEFTFRONT,
hl2ss.StreamPort.RM_VLC_LEFTLEFT,
hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
#hl2ss.StreamPort.RM_DEPTH_AHAT,
hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
hl2ss.StreamPort.RM_IMU_GYROSCOPE,
hl2ss.StreamPort.RM_IMU_MAGNETOMETER,
hl2ss.StreamPort.PERSONAL_VIDEO,
hl2ss.StreamPort.MICROPHONE,
hl2ss.StreamPort.SPATIAL_INPUT
'''



def display_vlc(data):
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    cv2.imshow('vlc', data.payload)
    cv2.waitKey(1)

def display_depth(data):
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    cv2.imshow('depth', data.payload.depth / np.max(data.payload.depth))
    cv2.imshow('ab', data.payload.ab / np.max(data.payload.ab))
    cv2.waitKey(1)

def display_imu(data):
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    imu_data = hl2ss.unpack_rm_imu(data.payload)
    count = imu_data.get_count()
    sample = imu_data.get_frame(0)
    print(f'Got {count} samples at time {data.timestamp}, first sample is (ticks = {sample.vinyl_hup_ticks} | {sample.soc_ticks}, x = {sample.x}, y = {sample.y}, z = {sample.z}, temperature={sample.temperature})')

def display_pv(data):
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    print('Focal length')
    print(data.payload.focal_length)
    print('Principal point')
    print(data.payload.principal_point)
    cv2.imshow('vlc', data.payload.image)
    cv2.waitKey(1)

microphone_buffer = np.empty((1,0), dtype=np.float32)

def display_microphone(data):
    global microphone_buffer
    print(f'Samples at time {data.timestamp}')
    microphone_buffer = np.hstack((microphone_buffer, hl2ss_utilities.microphone_planar_to_packed(data.payload)))

def display_si(data):
    si = hl2ss.unpack_si(data.payload)

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
        pose = hand_left.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
        print('Left wrist pose')
        print(pose.orientation)
        print(pose.position)
        print(pose.radius)
        print(pose.accuracy)
    else:
        print('No left hand data')

    if (si.is_valid_hand_right()):
        hand_right = si.get_hand_right()
        pose = hand_right.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
        print('Right wrist pose')
        print(pose.orientation)
        print(pose.position)
        print(pose.radius)
        print(pose.accuracy)
    else:
        print('No right hand data')


display_method = {
    hl2ss.StreamPort.RM_VLC_LEFTFRONT : display_vlc,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT : display_vlc,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT : display_vlc,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT : display_vlc,
    hl2ss.StreamPort.RM_DEPTH_AHAT : display_depth,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW : display_depth,
    hl2ss.StreamPort.RM_IMU_ACCELEROMETER : display_imu,
    hl2ss.StreamPort.RM_IMU_GYROSCOPE : display_imu,
    hl2ss.StreamPort.RM_IMU_MAGNETOMETER : display_imu,
    hl2ss.StreamPort.PERSONAL_VIDEO : display_pv,
    hl2ss.StreamPort.MICROPHONE : display_microphone,
    hl2ss.StreamPort.SPATIAL_INPUT : display_si
}

filename = os.path.join(path, f'{hl2ss.get_port_name(port)}.bin')
reader = hl2ss_io.create_rd(True, filename, hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')
reader.open()

while (True):
    data = reader.read()
    if (data is None):
        break
    display_method[port](data)

reader.close()

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paFloat32, channels=hl2ss.Parameters_MICROPHONE.CHANNELS, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE, output=True)
stream.start_stream()
stream.write(microphone_buffer.tobytes())
stream.stop_stream()
stream.close()


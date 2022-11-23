#------------------------------------------------------------------------------
# This script receives IMU samples from the HoloLens and prints them.
# Sensor details:
# Accelerometer:  93 samples per frame, sample rate ~1100 Hz
# Gyroscope:     315 samples per frame, sample rate ~6550 Hz
# Magnetometer:   11 samples per frame, sample rate   ~50 Hz
# The streams support three operating modes: 0) samples, 1) samples + rig pose,
# 2) query calibration (single transfer), except for the magnetometer stream
# which does not support mode 2. Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
# Options:
# hl2ss.StreamPort.RM_IMU_ACCELEROMETER
# hl2ss.StreamPort.RM_IMU_GYROSCOPE
# hl2ss.StreamPort.RM_IMU_MAGNETOMETER
port = hl2ss.StreamPort.RM_IMU_ACCELEROMETER

# Maximum bytes to receive per step
# Options:
# hl2ss.ChunkSize.RM_IMU_ACCELEROMETER
# hl2ss.ChunkSize.RM_IMU_GYROSCOPE
# hl2ss.ChunkSize.RM_IMU_MAGNETOMETER
chunk_size = hl2ss.ChunkSize.RM_IMU_ACCELEROMETER

# Operating mode
# 0: samples
# 1: samples + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_imu(host, port)
    print('Calibration data')
    print(data.extrinsics)
    quit()

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

client = hl2ss.rx_rm_imu(host, port, chunk_size, mode)
client.open()

while (enable):
    data = client.get_next_packet()
    print('Pose at time {ts}'.format(ts=data.timestamp))
    print(data.pose)
    imu_data = hl2ss.unpack_rm_imu(data.payload)
    count = imu_data.get_count()
    sample = imu_data.get_frame(0)
    print('Got {count} samples at time {ts}, first sample is (ticks = {st}, x = {x}, y = {y}, z = {z})'.format(count=count, ts=data.timestamp, st=sample.sensor_ticks_ns, x=sample.x, y=sample.y, z=sample.z))

client.close()
listener.join()

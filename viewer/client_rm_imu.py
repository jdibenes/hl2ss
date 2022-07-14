#------------------------------------------------------------------------------
# This script receives IMU samples from the HoloLens and prints them.
# Sensor details:
# Accelerometer:  93 samples per frame, sample rate ~1100 Hz
# Gyroscope:     315 samples per frame, sample rate ~6550 Hz
# Magnetometer:   11 samples per frame, sample rate   ~50 Hz
# The streams support three operating modes: 0) samples, 1) samples + rig pose,
# 2) query calibration (single transfer), except for the magnetometer stream
# which does not support mode 2.
#------------------------------------------------------------------------------

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port
# Options:
# hl2ss.StreamPort.RM_IMU_ACCELEROMETER
# hl2ss.StreamPort.RM_IMU_GYROSCOPE
# hl2ss.StreamPort.RM_IMU_MAGNETOMETER
port = hl2ss.StreamPort.RM_IMU_ACCELEROMETER

# Operating mode
# 0: samples
# 1: samples + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_imu(host, port)
    if (data is None):
        print('This stream does not support mode 2')
    else:
        print('Calibration data')
        print(data.extrinsics)
    quit()

pose_printer = hl2ss.pose_printer(24)
client = hl2ss.connect_client_rm_imu(host, port, 1024, mode)

try:
    while True:
        data = client.get_next_packet()
        timestamp = data.timestamp
        imu_data = hl2ss.unpacker_rm_imu(data.payload)

        pose_printer.push(timestamp, data.pose)

        # Print number of samples received (always > 0) and the first sample in the batch
        sample = imu_data.get_sample(0)
        print('Got {count} samples at time {ts}, first sample is (ticks = {st}, x = {x}, y = {y}, z = {z})'.format(count=imu_data.get_count(), ts=timestamp, st=sample.sensor_ticks_ns, x=sample.x, y=sample.y, z=sample.z))
except:
    pass

client.close()

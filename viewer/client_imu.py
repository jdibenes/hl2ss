# OK

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.15"

# Port number
# hl2ss.StreamPort.RM_IMU_ACCELEROMETER ( 93 samples per frame 1097 Hz)
# hl2ss.StreamPort.RM_IMU_GYROSCOPE     (315 samples per frame 6550 Hz)
# hl2ss.StreamPort.RM_IMU_MAGNETOMETER  ( 11 samples per frame 50 Hz)
port = hl2ss.StreamPort.RM_IMU_ACCELEROMETER

# Operating mode
# 0: samples
# 1: samples + rig pose
# 2: query extrinsics (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Maximum number of bytes to read from the socket buffer per step
# Use an appropriate power-of-two value
chunk_size = 1024

# Pose period
# Display pose every 'pose_period' frames if streaming in mode 1
# Set to 1 to display pose for each frame
pose_period = 10

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.get_mode2_rm_imu(host, port)
    if (data is None):
        print('This stream does not support mode 2')
    else:
        print('Extrinsics')
        print(data.extrinsics)

    quit()

client = hl2ss.gatherer()
pose_counter = hl2ss.counter()

client.open(host, port, chunk_size, mode)
client.configure(hl2ss.create_configuration_for_mode(mode))

while True:
    data = client.get_next_packet()
    timestamp = data.timestamp
    imu_data = hl2ss.unpacker_rm_imu(data.payload)

    # Print pose if streaming in mode 1
    if ((mode == hl2ss.StreamMode.MODE_1) and (pose_counter.increment() >= pose_period)):
        pose_counter.reset()
        print('Pose at {timestamp}'.format(timestamp=timestamp))
        print(data.pose)

    # Print the number of samples received and the first sample in the batch
    print('Got {count} samples'.format(count=imu_data.get_count()))
    print('First sample')
    print(imu_data.get_sample(0))

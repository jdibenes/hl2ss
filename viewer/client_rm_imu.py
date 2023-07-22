#------------------------------------------------------------------------------
# This script receives IMU samples from the HoloLens and prints them.
# Sensor details:
# Accelerometer:  93 samples per frame, sample rate ~1100 Hz effective ~12 Hz
# Gyroscope:     315 samples per frame, sample rate ~7500 Hz effective ~24 Hz
# Magnetometer:   11 samples per frame, sample rate   ~50 Hz effective  ~5 Hz
# The streams support three operating modes: 0) samples, 1) samples + rig pose,
# 2) query calibration (single transfer), except for the magnetometer stream
# which does not support mode 2.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import configparser

# Settings --------------------------------------------------------------------

# HoloLens address
host = config['DEFAULT']['ip']

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
    print('Extrinsics')
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
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    imu_data = hl2ss.unpack_rm_imu(data.payload)
    count = imu_data.get_count()
    sample = imu_data.get_frame(0)
    print(f'Got {count} samples at time {data.timestamp}')
    print(f'First sample: sensor_ticks={sample.vinyl_hup_ticks} soc_ticks={sample.soc_ticks} x={sample.x} y={sample.y} z={sample.z} temperature={sample.temperature}')

client.close()
listener.join()

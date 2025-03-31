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
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

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
    data = hl2ss_lnm.download_calibration_rm_imu(host, port)
    print('Calibration data')
    print('Extrinsics')
    print(data.extrinsics)
    quit()


listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

expected_count = hl2ss.rm_imu_get_batch_size(port)

client = hl2ss_lnm.rx_rm_imu(host, port, mode=mode)
client.open()

while (not listener.pressed()):
    data = client.get_next_packet()

    imu = data.payload

    count        = imu.count
    sensor_ticks = imu.vinyl_hup_ticks
    soc_ticks    = imu.soc_ticks
    x            = imu.x
    y            = imu.y
    z            = imu.z
    temperature  = imu.temperature

    print(f'Got {count}/{expected_count} samples at time {data.timestamp}')
    print(f'First sample: sensor_ticks={sensor_ticks[ 0]} soc_ticks={soc_ticks[ 0]} x={x[ 0]} y={y[ 0]} z={z[ 0]} temperature={temperature[ 0]}')
    print(f'...')
    print(f'Last  sample: sensor_ticks={sensor_ticks[-1]} soc_ticks={soc_ticks[-1]} x={x[-1]} y={y[-1]} z={z[-1]} temperature={temperature[-1]}')
    print(f'Pose')
    print(data.pose)

client.close()
listener.close()

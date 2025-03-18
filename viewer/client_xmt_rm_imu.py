#------------------------------------------------------------------------------
# C++ Multithreaded Client Python Extension Test.
# Requires building the hl2ss_ulm_stream extension.
#
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

import time
import hl2ss
import hl2ss_lnm
import hl2ss_mt

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

# Buffer size (batches)
buffer_size = 20

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss_lnm.download_calibration_rm_imu(host, port)
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

configuration = hl2ss_mt.create_configuration(port)
configuration['mode'] = mode

client = hl2ss_mt.rx_rm_imu(host, port, buffer_size, configuration)
client.open()

frame_index = -1

while (enable):
    data = client.get_by_index(frame_index)
    if (data.status == hl2ss_mt.Status.WAIT):
        time.sleep(1/1000)
        continue
    elif (data.status == hl2ss_mt.Status.DISCARDED):
        frame_index = -1
        continue

    frame_index = data.frame_stamp + 1

    imu_data = hl2ss.unpack_rm_imu(data.payload)
    count = imu_data.get_count()
    sample = imu_data.get_frame(0)
    
    print(f'Got {count} samples at time {data.timestamp}')
    print(f'First sample: sensor_ticks={sample.vinyl_hup_ticks} soc_ticks={sample.soc_ticks} x={sample.x} y={sample.y} z={sample.z} temperature={sample.temperature}')
    print(f'Pose')
    print(data.pose)

client.close()
listener.join()


import multiprocessing as mp
import time
import os
import hl2ss
import hl2ss_io
import hl2ss_mp

host = '192.168.1.7'
path = './data'

# Ports
ports = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    #hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    #hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    #hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    #hl2ss.StreamPort.RM_DEPTH_AHAT,
    #hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    #hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
    #hl2ss.StreamPort.RM_IMU_GYROSCOPE,
    #hl2ss.StreamPort.RM_IMU_MAGNETOMETER,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    hl2ss.StreamPort.MICROPHONE,
    #hl2ss.StreamPort.SPATIAL_INPUT
    ]

# RM VLC parameters
vlc_mode    = hl2ss.StreamMode.MODE_1
vlc_profile = hl2ss.VideoProfile.H264_BASE
vlc_bitrate = 1*1024*1024

# RM Depth AHAT parameters
ahat_mode = hl2ss.StreamMode.MODE_1
ahat_profile = hl2ss.VideoProfile.H264_BASE
ahat_bitrate = 8*1024*1024

# RM Depth Long Throw parameters
lt_mode = hl2ss.StreamMode.MODE_1
lt_filter = hl2ss.PngFilterMode.Paeth

# RM IMU parameters
imu_mode = hl2ss.StreamMode.MODE_1

# PV parameters
pv_mode = hl2ss.StreamMode.MODE_1
pv_width = 760
pv_height = 428
pv_framerate = 30
pv_profile = hl2ss.VideoProfile.H264_BASE
pv_bitrate = 1*1024*1024
pv_format = 'bgr24'

# Maximum number of frames in buffer
buffer_elements = 300

if __name__ == '__main__':
    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    producer = hl2ss_mp.producer()
    producer.configure_rm_vlc(False, host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(False, host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(False, host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_vlc(False, host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_depth_ahat(False, host, hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss.ChunkSize.RM_DEPTH_AHAT, ahat_mode, ahat_profile, ahat_bitrate)
    producer.configure_rm_depth_longthrow(False, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, lt_mode, lt_filter)
    producer.configure_rm_imu(host, hl2ss.StreamPort.RM_IMU_ACCELEROMETER, hl2ss.ChunkSize.RM_IMU_ACCELEROMETER, imu_mode)
    producer.configure_rm_imu(host, hl2ss.StreamPort.RM_IMU_GYROSCOPE, hl2ss.ChunkSize.RM_IMU_GYROSCOPE, imu_mode)
    producer.configure_rm_imu(host, hl2ss.StreamPort.RM_IMU_MAGNETOMETER, hl2ss.ChunkSize.RM_IMU_MAGNETOMETER, imu_mode)
    producer.configure_pv(False, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, pv_mode, pv_width, pv_height, pv_framerate, pv_profile, pv_bitrate, pv_format)
    producer.configure_microphone(False, host, hl2ss.StreamPort.MICROPHONE, hl2ss.ChunkSize.MICROPHONE, hl2ss.AudioProfile.AAC_24000)
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)

    for port in ports:
        producer.initialize(port, buffer_elements)
        producer.start(port)

    writers = {}
    filenames = {}

    for port in ports:
        filenames[port] = os.path.join(path, f'{hl2ss.get_port_name(port)}.bin')
        writers[port] = hl2ss_io.wr_process_producer(filenames[port], producer, port, 'MULTI RECORDER TEST'.encode())
        writers[port].start()

    time.sleep(20)

    for port in ports:
        writers[port].stop()

    for port in ports:
        writers[port].join()

    for port in ports:
        producer.stop(port)

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)


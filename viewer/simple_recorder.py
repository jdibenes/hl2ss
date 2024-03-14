#------------------------------------------------------------------------------
# Recording example. Data is recorded to binary files. See simple player for
# how to extract recorded data.
# Press space to start recording.
# Press stop to stop recording.
#------------------------------------------------------------------------------

from pynput import keyboard

import os
import threading
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Output directory
path = './data'

# Unpack to viewable formats (e.g., encoded video to mp4)
unpack = True

# Ports to record
ports = [
    #hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    #hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    #hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    #hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    #hl2ss.StreamPort.RM_DEPTH_AHAT,
    #hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    #hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
    #hl2ss.StreamPort.RM_IMU_GYROSCOPE,
    #hl2ss.StreamPort.RM_IMU_MAGNETOMETER,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    #hl2ss.StreamPort.MICROPHONE,
    #hl2ss.StreamPort.SPATIAL_INPUT,
    #hl2ss.StreamPort.EXTENDED_EYE_TRACKER,
    hl2ss.StreamPort.EXTENDED_AUDIO,
    ]

# PV parameters
pv_width     = 760
pv_height    = 428
pv_framerate = 30

# EET parameters
eet_fps = 30 # 30, 60, 90

# Maximum number of frames in buffer
buffer_elements = 300

#------------------------------------------------------------------------------

if __name__ == '__main__':
    if ((hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports) and (hl2ss.StreamPort.RM_DEPTH_AHAT in ports)):
        print('Error: Simultaneous RM Depth Long Throw and RM Depth AHAT streaming is not supported. See known issues at https://github.com/jdibenes/hl2ss.')
        quit()

    # Keyboard events ---------------------------------------------------------
    start_event = threading.Event()
    stop_event = threading.Event()

    def on_press(key):
        global start_event
        global stop_event

        if (key == keyboard.Key.space):
            start_event.set()
        elif (key == keyboard.Key.esc):
            stop_event.set()

        return not stop_event.is_set()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Start PV Subsystem if PV is selected ------------------------------------
    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start receivers ---------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, decoded=False))
    producer.configure(hl2ss.StreamPort.RM_IMU_ACCELEROMETER, hl2ss_lnm.rx_rm_imu(host, hl2ss.StreamPort.RM_IMU_ACCELEROMETER))
    producer.configure(hl2ss.StreamPort.RM_IMU_GYROSCOPE, hl2ss_lnm.rx_rm_imu(host, hl2ss.StreamPort.RM_IMU_GYROSCOPE))
    producer.configure(hl2ss.StreamPort.RM_IMU_MAGNETOMETER, hl2ss_lnm.rx_rm_imu(host, hl2ss.StreamPort.RM_IMU_MAGNETOMETER))
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format=None))
    producer.configure(hl2ss.StreamPort.MICROPHONE, hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, decoded=False))
    producer.configure(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))
    producer.configure(hl2ss.StreamPort.EXTENDED_EYE_TRACKER, hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps))
    producer.configure(hl2ss.StreamPort.EXTENDED_AUDIO, hl2ss_lnm.rx_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO, decoded=False))

    for port in ports:
        producer.initialize(port, buffer_elements)
        producer.start(port)

    # Wait for start signal ---------------------------------------------------
    print('Press space to start recording...')
    start_event.wait()
    print('Preparing...')

    # Start writers -----------------------------------------------------------
    filenames = {port : os.path.join(path, f'{hl2ss.get_port_name(port)}.bin') for port in ports}
    writers = {port : hl2ss_utilities.wr_process_producer(filenames[port], producer, port, 'hl2ss simple recorder'.encode()) for port in ports}
    
    for port in ports:
        writers[port].start()

    # Wait for stop signal ----------------------------------------------------
    print('Recording started.')
    print('Press esc to stop recording...')
    stop_event.wait()
    print('Stopping...')

    # Stop writers and receivers ----------------------------------------------
    for port in ports:
        writers[port].stop()

    for port in ports:
        writers[port].join()

    for port in ports:
        producer.stop(port)

    # Stop PV Subsystem if PV is selected -------------------------------------
    if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
        hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

    # Quit if binaries are not to be unpacked ---------------------------------
    if (not unpack):
        quit()

    print('Unpacking binaries (may take several minutes)...')

    # Unpack encoded video streams to a single MP4 file -----------------------
    ports_to_mp4 = [
        hl2ss.StreamPort.PERSONAL_VIDEO,
        hl2ss.StreamPort.MICROPHONE,
        hl2ss.StreamPort.EXTENDED_AUDIO,
        hl2ss.StreamPort.RM_VLC_LEFTFRONT,
        hl2ss.StreamPort.RM_VLC_LEFTLEFT,
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
        hl2ss.StreamPort.RM_DEPTH_AHAT,        
    ]

    mp4_input_filenames = [filenames[port] for port in ports_to_mp4 if (port in ports)]
    mp4_output_filename = os.path.join(path, 'video.mp4')

    if (len(mp4_input_filenames) > 0):
        hl2ss_utilities.unpack_to_mp4(mp4_input_filenames, mp4_output_filename)

    # Unpack RM Depth Long Throw to a tar file containing Depth and AB PNGs ---
    if (hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports):
        hl2ss_utilities.unpack_to_png(filenames[hl2ss.StreamPort.RM_DEPTH_LONGTHROW], os.path.join(path, 'long_throw.tar'))

    # Unpack stream metadata and numeric payloads to csv ----------------------
    for port in ports:
        input_filename = filenames[port]
        output_filename = input_filename[:input_filename.rfind('.bin')] + '.csv'
        hl2ss_utilities.unpack_to_csv(input_filename, output_filename)


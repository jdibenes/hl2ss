#------------------------------------------------------------------------------
# This script receives video from the HoloLens front RGB camera and plays it.
# The camera supports various resolutions and framerates. See
# https://github.com/jdibenes/hl2ss/blob/main/etc/pv_configurations.txt
# for a list of supported formats. The default configuration is 1080p 30 FPS. 
# The stream supports three operating modes: 0) video, 1) video + camera pose, 
# 2) query calibration (single transfer).
# Press esc to stop.
#------------------------------------------------------------------------------
from PIL import Image
from io import BytesIO
from pynput import keyboard
import time
import calendar
import os
import cv2
import hl2ss_imshow
import hl2ss
#<<<<<<< HEAD:viewer/client_pv.py
import configparser
#=======
import hl2ss_lnm
#>>>>>>> 5d92301451f23c976ebcf6f65a35728896a2bb09:viewer/client_stream_pv.py

#save = True
#save_path = './capture_frames/pv/'
def get_pv_image():
    image = None
# Settings --------------------------------------------------------------------
    config = configparser.ConfigParser()
    config.read('config.ini')
# HoloLens address
    host = config['DEFAULT']['ip']

# Operating mode
# 0: video
# 1: video + camera pose
# 2: query calibration (single transfer)
    mode = hl2ss.StreamMode.MODE_1

# Enable Mixed Reality Capture (Holograms)
    enable_mrc = True


# Camera paramers
    width     = 1920
    height    = 1080
    framerate = 15

# Framerate denominator (must be > 0)
# Effective FPS is framerate / divisor
    divisor = 1 

# Video encoding profile
    profile = hl2ss.VideoProfile.RAW

# Decoded format
# Options include:
# 'bgr24'
# 'rgb24'
# 'bgra'
# 'rgba'
# 'gray8'
    decoded_format = 'bgr24'

#------------------------------------------------------------------------------

    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, enable_mrc=enable_mrc)

    if (mode == hl2ss.StreamMode.MODE_2):
        data = hl2ss_lnm.download_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width, height, framerate)
        print('Calibration')
        print(f'Focal length: {data.focal_length}')
        print(f'Principal point: {data.principal_point}')
        print(f'Radial distortion: {data.radial_distortion}')
        print(f'Tangential distortion: {data.tangential_distortion}')
        print('Projection')
        print(data.projection)
        print('Intrinsics')
        print(data.intrinsics)
    else:
        enable = True
        #listener = keyboard.Listener(on_press=on_press)
        #listener.start()

        client = hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, mode=mode, width=width, height=height, framerate=framerate, divisor=divisor, profile=profile, decoded_format=decoded_format)
        client.open()

        while (enable):
            data = client.get_next_packet()

            print(f'Pose at time {data.timestamp}')
            print(data.pose)
            print(f'Focal length: {data.payload.focal_length}')
            print(f'Principal point: {data.payload.principal_point}')

            #cv2.imshow('Video', data.payload.image)
            #if save:
                #gmt = time.gmtime()
                #cv2.imwrite(os.path.join(save_path, str(calendar.timegm(gmt)) + '.png'), data.payload.image)
            image = data.payload.image
            color_coverted = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(color_coverted)
            break
            #cv2.waitKey(1)

        client.close()
        #listener.join()

    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
    output = BytesIO()
    image.convert("RGB").save(output, "BMP")
    data = output.getvalue()[14:]
    output.close()

    return data

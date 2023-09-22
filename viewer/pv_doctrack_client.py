#------------------------------------------------------------------------------
# This script receives video from the HoloLens front RGB camera and plays it.
# The camera supports various resolutions and framerates. See
# https://github.com/jdibenes/hl2ss/blob/main/etc/pv_configurations.txt
# for a list of supported formats. The default configuration is 1080p 30 FPS. 
# The stream supports three operating modes: 0) video, 1) video + camera pose, 
# 2) query calibration (single transfer).
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard
import sys
sys.path.append("EAST")
import cv2
import hl2ss_imshow
import hl2ss
import configparser
from detect import *
#import docdetect

# Settings --------------------------------------------------------------------
config = configparser.ConfigParser()
config.read('config.ini')
# HoloLens address
host = config['DEFAULT']['ip']

# Port
port = hl2ss.StreamPort.PERSONAL_VIDEO

# Operating mode
# 0: video
# 1: video + camera pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Camera parameters
width     = 1920
height    = 1080
framerate = 15

# Video encoding profile
profile = hl2ss.VideoProfile.H265_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = hl2ss.get_video_codec_bitrate(width, height, framerate, hl2ss.get_video_codec_default_factor(profile))

# Decoded format
# Options include:
# 'bgr24'
# 'rgb24'
# 'bgra'
# 'rgba'
# 'gray8'
decoded_format = 'bgr24'
def check_empty_img(image):
    # Reading Image
    # You can give path to the 
    # image as first argument
    ##image = cv2.imread(img)
  
    # Checking if the image is empty or not
    if image is None:
        return True
        result = "Image is empty!!"
    else:
        return False
        result = "Image is not empty!!"
  
    return result
      


#------------------------------------------------------------------------------

hl2ss.start_subsystem_pv(host, port)

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_pv(host, port, width, height, framerate)
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

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    client = hl2ss.rx_decoded_pv(host, port, hl2ss.ChunkSize.PERSONAL_VIDEO, mode, width, height, framerate, profile, bitrate, decoded_format)
    client.open()
    #img_path    = '../ICDAR_2015/test_img/img_2.jpg'
    model_path  = './EAST/pths/east_vgg16.pth'
    #res_img     = './res.bmp'
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(device)
    model = EAST().to(device)
    model.load_state_dict(torch.load(model_path))
    model.eval()
    #img = Image.open(img_path)
    while (enable):
        data = client.get_next_packet()

        print(f'Pose at time {data.timestamp}')
        print(data.pose)
        print(f'Focal length: {data.payload.focal_length}')
        print(f'Principal point: {data.payload.principal_point}')
        #if data.payload.image:
        #print(data.payload.image.shape)
        if True:
            #img = cv2.resize(data.payload.image, (640, 360),  interpolation=cv2.INTER_AREA)

            img = data.payload.image
            if check_empty_img(img):
                continue
            img = cv2_to_pil(img)
            boxes = detect(img, model, device)
            plot_img = plot_boxes(img, boxes)
            cv2.imshow('frame', pil_to_cv2(plot_img))
        
            #rects = docdetect.process(frame)

            #frame = docdetect.draw(rects, frame)
            #cv2.imshow('Video', frame)
        #cv2.imshow('Video', data.payload.image)
            cv2.waitKey(1)
    
    client.close()
    listener.join()

hl2ss.stop_subsystem_pv(host, port)

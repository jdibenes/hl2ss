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
import pickle
import cv2
import hl2ss_imshow
import hl2ss
import configparser
import sys
#import pytesseract
import easyocr
model = easyocr.Reader(['en']) # this needs to run only once to load the model into memory

sys.path.append("./HandMovementTracking/")
#import sys
#sys.path.append("EAST")
#from detect import *
import numpy as np
#from doctr.models import ocr_predictor
from collections import deque
import mediapipe as mp
from utils.utils_v2 import get_idx_to_coordinates, rescale_frame

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
#model = ocr_predictor(det_arch='db_resnet50', reco_arch='crnn_vgg16_bn', pretrained=True)

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
mode = hl2ss.StreamMode.MODE_0

# Camera parameters
width     = 1920
height    = 1080
framerate = 15
# Video encoding profile
profile = hl2ss.VideoProfile.H264_MAIN
#image = 
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

def padding(img, pad):
    old_image_height, old_image_width, channels = img.shape

# create new image of desired size and color (blue) for padding
    new_image_width = old_image_width + pad
    new_image_height = old_image_height + pad
    color = (255,255,255)
    result = np.full((new_image_height,new_image_width, channels), color, dtype=np.uint8)

# compute center offset
    x_center = (new_image_width - old_image_width) // 2
    y_center = (new_image_height - old_image_height) // 2

# copy img image into center of result image
    result[y_center:y_center+old_image_height, 
        x_center:x_center+old_image_width] = img
    return result
#------------------------------------------------------------------------------
hands = mp_hands.Hands(
        min_detection_confidence=0.5, min_tracking_confidence=0.5)
hand_landmark_drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=2)
    #hand_connection_drawing_spec = mp_drawing.Drawi/hl2ss/viewer/hl2ss.py", line 609, in get_next_packet
   #pts = deque(maxlen=64)
#model_path  = './EAST/pths/east_vgg16.pth'
    #res_img     = './res.bmp'
#device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
#print(device)
#model = EAST().to(device)
#model.load_state_dict(torch.load(model_path))
#model.eval()

old_4 = [0,0]
old_8 = [0,0]
count_4 = 0
count_8 = 0
xmin = 0
xmax = 0
ymin = 0
ymax = 0
mode_ = 1
cache_image = None
release = False
def check_finger_movement(new, old, count, thresh=10):
    
    y = abs(new[0] - old[0])
    x = abs(new[1] - old[1]) 
    if x <= 7 and y <= 7:
        return count+1
    return 0

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
 
def crop_image(img, px, py, w, h):
    x1 = max(px - int(w), 0)
    y1 = max(py - (int(h*2)), 0)
    x2 = min(px + int(w), img.shape[1])
    y2 = min(py, img.shape[0])

    crop_img = img[y1:y2, x1:x2]
    return crop_img

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
    while (enable):
        data = client.get_next_packet()

        #print(f'Pose at time {data.timestamp}')
        #print(data.pose)
        #print(f'Focal length: {data.payload.focal_length}')
        #print(f'Principal point: {data.payload.principal_point}')


        idx_to_coordinates = {}
        image = data.payload.image 
        if check_empty_img(image):
            continue
 
        #image = rescale_frame(image, percent=50)
        #mage = cv2.flip(image, 1)
        dim = (640, 360)
        resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
      
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        results_hand = hands.process(resized)
        resized.flags.writeable = True
        resized = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
       
        if results_hand.multi_hand_landmarks:
            for hand_landmarks in results_hand.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                        image=resized,
                        landmark_list=hand_landmarks,
                        #connections=mp_hands.HAND_CONNECTIONS,
                        landmark_drawing_spec=hand_landmark_drawing_spec,
                        #connection_drawing_spec=hand_connection_drawing_spec
                        )
                idx_to_coordinates = get_idx_to_coordinates(resized, results_hand)
                
        if mode_ == 2:
            cv2.imshow('Video', image)
            cv2.waitKey(1)

            continue
        if mode_ == 1 and 8 in idx_to_coordinates:
            #count_4 = check_finger_movement(idx_to_coordinates[4], old_4, count_4)
            count_8 = check_finger_movement(idx_to_coordinates[8], old_8, count_8)
            #old_4 = idx_to_coordinates[4]
    
            old_8 = idx_to_coordinates[8] 

            if count_8 >= 3:
                print('stable')
                
                count_8 = 0
            #pts.appendleft(idx_to_coordinates[8])  # Index Finger
        #for i in range(1, len(pts)):
        #    if pts[i - 1] is None or pts[i] is None:
        #        continue
        #    thick = int(np.sqrt(len(pts) / float(i + 1)) * 4.5)
        #   cv2.line(image, pts[i - 1], pts[i], (0, 255, 0), thick)
                #xmin = min(idx_to_coordinates[8][1], idx_to_coordinates[4][1])
                #xmax = max(idx_to_coordinates[8][1], idx_to_coordinates[4][1])
                #ymin = min(idx_to_coordinates[8][0], idx_to_coordinates[4][0])
                #ymax = max(idx_to_coordinates[8][0], idx_to_coordinates[4][0])
                
                #release = True
        
                #release = False
                start_point = (idx_to_coordinates[8][0]-50,idx_to_coordinates[8][1]-100)
                end_point = (idx_to_coordinates[8][0]+50,idx_to_coordinates[8][1])
                color=(0,0,255)
                thickness=2
                #image = cache_image

                #image = cv2.rectangle(image, start_point, end_point, color, thickness)
                doc = image[start_point[1]:end_point[1], start_point[0]:end_point[0]]
                doc = padding(doc,50)
                print(model.readtext(doc))
                #print(pytesseract.image_to_string(doc))
                #img = image[xmin:xmax, ymin:ymax]
                #mg = data.payload.image
            #if check_empty_img(img):
            #    continue
                #if img.shape[1] == 0 or img.shape[0] == 0:
                #    continue
                #img = cv2_to_pil(img)
                #boxes = detect(img, model, device)
                #plot_img = plot_boxes(img, boxes)
                cv2.imshow('frame', doc) 
         #   cv2.imshow("Res", image)
                cv2.waitKey(1)
        
        if mode_ == 0 and 8 in idx_to_coordinates and 4 in idx_to_coordinates:
            count_4 = check_finger_movement(idx_to_coordinates[4], old_4, count_4)
            count_8 = check_finger_movement(idx_to_coordinates[8], old_8, count_8)
            old_4 = idx_to_coordinates[4]
            old_8 = idx_to_coordinates[8]
            if count_4 >= 5 and count_8 >= 5:
                print('stable')
                count_4 = 0
                count_8 = 0
            #pts.appendleft(idx_to_coordinates[8])  # Index Finger
        #for i in range(1, len(pts)):
        #    if pts[i - 1] is None or pts[i] is None:
        #        continue
        #    thick = int(np.sqrt(len(pts) / float(i + 1)) * 4.5)
        #   cv2.line(image, pts[i - 1], pts[i], (0, 255, 0), thick)
                xmin = min(idx_to_coordinates[8][1], idx_to_coordinates[4][1])
                xmax = max(idx_to_coordinates[8][1], idx_to_coordinates[4][1])
                ymin = min(idx_to_coordinates[8][0], idx_to_coordinates[4][0])
                ymax = max(idx_to_coordinates[8][0], idx_to_coordinates[4][0])
                
                #release = True
        
                #release = False
                start_point = (ymin,xmin)
                end_point = (ymax,xmax)
                color=(0,0,255)
                thickness=2
                #image = cache_image
                image = cv2.rectangle(image, start_point, end_point, color, thickness)
                doc = image[xmin:xmax, ymin:ymax]
                doc = padding(doc,50)
                print(model.readtext(doc))



                #img = image[xmin:xmax, ymin:ymax]
                #mg = data.payload.image
            #if check_empty_img(img):
            #    continue
                #if img.shape[1] == 0 or img.shape[0] == 0:
                #    continue
                #img = cv2_to_pil(img)
                #boxes = detect(img, model, device)
                #plot_img = plot_boxes(img, boxes)
                cv2.imshow('frame', image)
                    
         #   cv2.imshow("Res", image)
                cv2.waitKey(1)
        #elif 8 not in idx_to_coordinates and 4 not in idx_to_coordinates:
        #    cache_image = image
        #if cv2.waitKey(5) & 0xFF == 27:
        #    break
    #hands.close()
        #cv2.imshow('res', image)
        #cv2.waitKey(1)
        #cv2.imshow('Video', data.payload.image)
        #cv2.waitKey(1)
    hands.close()
    client.close()
    listener.join()

hl2ss.stop_subsystem_pv(host, port)

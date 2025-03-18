#------------------------------------------------------------------------------
# This script receives extended eye tracking data from the HoloLens and prints
# it.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import time
import hl2ss
import hl2ss_lnm
import hl2ss_mt

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Target Frame Rate
# Options: 30, 60, 90
framerate = 90

# Buffer size (packets)
buffer_size = 450

#------------------------------------------------------------------------------

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

configuration = hl2ss_mt.create_configuration(hl2ss.StreamPort.EXTENDED_EYE_TRACKER)
configuration['framerate'] = framerate

client = hl2ss_mt.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, buffer_size, configuration)
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

    eet = hl2ss.unpack_eet(data.payload)

    # See
    # https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/extended-eye-tracking-native
    # for details

    print(f'Tracking status at time {data.timestamp}')
    print('Pose')
    print(data.pose)
    print(f'Calibration: Valid={eet.calibration_valid}')
    print(f'Combined eye gaze: Valid={eet.combined_ray_valid} Origin={eet.combined_ray.origin} Direction={eet.combined_ray.direction}')
    print(f'Left eye gaze: Valid={eet.left_ray_valid} Origin={eet.left_ray.origin} Direction={eet.left_ray.direction}')
    print(f'Right eye gaze: Valid={eet.right_ray_valid} Origin={eet.right_ray.origin} Direction={eet.right_ray.direction}')

    # "...not supported by HoloLens 2 at this time"
    print(f'Left eye openness: Valid={eet.left_openness_valid} Value={eet.left_openness}')
    print(f'Right eye openness: Valid={eet.right_openness_valid} Value={eet.right_openness}')
    print(f'Vergence distance: Valid={eet.vergence_distance_valid} Value={eet.vergence_distance}')

client.close()
listener.join()

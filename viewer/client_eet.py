#------------------------------------------------------------------------------
# This script receives extended eye tracking data from the HoloLens and prints
# it.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss

import logging

log = logging.getLogger(__name__)


# Settings --------------------------------------------------------------------

DEFAULT_KEY = "tcn/loc/hl2/*"
# most simple zenoh config for now
conf = {"mode": "peer", "queries_default_timeout": 10000}
eye_fps = 30
#------------------------------------------------------------------------------

enable = True

logging.basicConfig(level=logging.DEBUG)

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

with hl2ss.mgr_rpc_interface(conf, DEFAULT_KEY) as mgr:
    if not mgr.start_eet(eye_fps=eye_fps):
        log.error("Cannot Start Eye Tracking Sensor")
        exit(1)

client = hl2ss.rx_eet(conf, DEFAULT_KEY)
client.open()

while (enable):
    data = client.get_next_packet()
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

with hl2ss.mgr_rpc_interface(conf, DEFAULT_KEY) as mgr:
    if not mgr.stop_eet():
        log.error("Cannot Stop Eye Tracking Sensor")

#------------------------------------------------------------------------------
# This script receives video frames and extended eye tracking data from the 
# HoloLens. The received left, right, and combined gaze pointers are projected
# onto the video frame.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import cv2
import hl2ss
import hl2ss_mp
import zenoh
import logging

log = logging.getLogger(__name__)

DEFAULT_KEY = "tcn/loc/hl2/*/cfg/pv"

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    enable = True

    logging.basicConfig(level=logging.DEBUG)

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    zenoh.init_logger()

    # most simple zenoh config for now
    conf = {"mode": "peer"}

    # Start PV stream ------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure_pv(True, hl2ss.StreamType.PERSONAL_VIDEO, conf, DEFAULT_KEY, 'bgr24')
    producer.initialize(hl2ss.StreamType.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamType.PERSONAL_VIDEO)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamType.PERSONAL_VIDEO, manager, ...)
    sink_pv.get_attach_response()

    # Main Loop ---------------------------------------------------------------
    while enable:

        # Wait for PV frame ---------------------------------------------------
        sink_pv.acquire()

        # Get PV frame and nearest (in time) EET frame ------------------------
        _, data_pv = sink_pv.get_most_recent_frame()
        if (data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose)):
            print("pv invalid data or pose")
            continue

        image = data_pv.payload.image

        # Display frame -------------------------------------------------------
        if image is not None and image.size > 0:
            cv2.imshow('Video', image)
        else:
            log.warning("received empty image..")
        cv2.waitKey(1)

    # Stop PV stream -------------------------------------------------
    sink_pv.detach()
    producer.stop(hl2ss.StreamType.PERSONAL_VIDEO)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

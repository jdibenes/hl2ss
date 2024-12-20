#------------------------------------------------------------------------------
# Experimental simultaneous RM Depth AHAT and RM Depth Long Throw.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import time
import numpy as np
import multiprocessing as mp
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Buffer size in seconds
buffer_size = 5

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Start streams -----------------------------------------------------------
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT))
    producer.configure(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_AHAT, buffer_size * hl2ss.Parameters_RM_DEPTH_AHAT.FPS)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, buffer_size * hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS)

    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    # Without this delay, the depth streams might crash and require rebooting 
    # the HoloLens
    time.sleep(5) 
    producer.start(hl2ss.StreamPort.RM_DEPTH_AHAT)    

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_ht = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_AHAT, manager, None)
    sink_lt = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, None)

    sink_ht.get_attach_response()
    sink_lt.get_attach_response()

    # Main loop ---------------------------------------------------------------
    while (enable):
        _, data_ht = sink_ht.get_most_recent_frame()
        _, data_lt = sink_lt.get_most_recent_frame()

        if (data_ht is not None):
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_AHAT) + '-depth', cv2.applyColorMap(((data_ht.payload.depth / 1056) * 255).astype(np.uint8), cv2.COLORMAP_JET)) # Scaled for visibility
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_AHAT) + '-ab', np.sqrt(data_ht.payload.ab).astype(np.uint8)) # Scaled for visibility

        if (data_lt is not None):
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW) + '-depth', cv2.applyColorMap(((data_lt.payload.depth / 7500) * 255).astype(np.uint8), cv2.COLORMAP_JET)) # Scaled for visibility
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW) + '-ab', np.sqrt(data_lt.payload.ab).astype(np.uint8)) # Scaled for visibility

        cv2.waitKey(1)

    # Stop streams ------------------------------------------------------------
    sink_ht.detach()
    sink_lt.detach()

    producer.stop(hl2ss.StreamPort.RM_DEPTH_AHAT)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

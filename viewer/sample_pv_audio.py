#------------------------------------------------------------------------------
# Sample audio/video player.
# Press ESC to stop.
#------------------------------------------------------------------------------

import numpy as np
import time
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mx
import hl2ss_mp
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Video parameters
width = 1280
height = 720
framerate = 30

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Start PV Subsystem ------------------------------------------------------
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start PV and Extended Audio streams -------------------------------------
    sink_pv = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=width, height=height, framerate=framerate))
    sink_ea = hl2ss_mp.stream(hl2ss_lnm.rx_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO))
    
    sink_pv.open()
    sink_ea.open()

    fs_ea = -1

    # Setup Multimedia Player -------------------------------------------------
    player = hl2ss_utilities.audio_player(np.float32, True, hl2ss.Parameters_MICROPHONE.CHANNELS, hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)
    player.open()

    cv2.namedWindow('Video')
    
    # Main Loop ---------------------------------------------------------------
    # Stop if ESC is pressed
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Audio must be read sequentially
        # using get_most_recent_frame will result in audio glitches since 
        # get_most_recent_frame repeats or drops frames as necessary
        status, fs_ea, data_ea = sink_ea.get_buffered_frame(fs_ea)
        if (status == hl2ss_mx.Status.WAIT):
            time.sleep(0.008)
            continue
        if (status == hl2ss_mx.Status.DISCARDED):
            fs_ea = -1
            continue
        fs_ea += 1

        player.put(data_ea.timestamp, data_ea.payload)
        # Coarse audio/video synchronization based on audio frame timestamps
        player_timestamp = player.get_timestamp()
        if (player_timestamp > 0):
            # Sync to audio
            _, data_pv = sink_pv.get_nearest(player_timestamp, hl2ss_mx.TimePreference.PREFER_PAST)
            if (data_pv is not None):
                cv2.imshow('Video', data_pv.payload.image)

    # Stop PV and Extended Audio streams --------------------------------------
    sink_pv.close()
    sink_ea.close()

    # Stop PV subsystem -------------------------------------------------------
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Close Audio Player ------------------------------------------------------
    player.close()

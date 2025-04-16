#------------------------------------------------------------------------------
# Player example. Plays PV and Microphone data that was previously recorded 
# using simple recorder.
#------------------------------------------------------------------------------

import numpy as np
import cv2
import os
import hl2ss_imshow
import hl2ss
import hl2ss_io
import hl2ss_mx
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# Directory containing the recorded data
path = './data/DS-2025-04-16-17-45-07'

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Create readers ----------------------------------------------------------
    # Stream type is detected automatically
    rd_pv = hl2ss_io.sequencer(hl2ss_io.create_rd(os.path.join(path, f'{hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)}.bin'), hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24'))
    rd_mc = hl2ss_io.create_rd(os.path.join(path, f'{hl2ss.get_port_name(hl2ss.StreamPort.MICROPHONE)}.bin'), hl2ss.ChunkSize.SINGLE_TRANSFER, True)

    rd_pv.open()
    rd_mc.open()

    # Stream parameters are also stored in the bin files
    # No RAW audio support in this example
    if (rd_mc.profile == hl2ss.AudioProfile.RAW):
        print('This example does not support RAW audio')

    # Create audio player -----------------------------------------------------
    audio_subtype     = np.float32
    audio_planar      = True
    audio_channels    = hl2ss.Parameters_MICROPHONE.CHANNELS
    audio_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE

    player = hl2ss_utilities.audio_player(audio_subtype, audio_planar, audio_channels, audio_sample_rate)
    player.open()

    cv2.namedWindow('Video')

    # Render decoded contents -------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Reader get_next_packet() behaves almost the same as get_next_packet() for
        # receivers (rx_[...]) except that it returns None on end of file
        if (player.pending() <= player.buffer_frames):
            data_mc = rd_mc.get_next_packet()
            if (data_mc is not None):
                # Reader packets have the same structure as receiver packets
                # Send audio data to player
                player.put(data_mc.timestamp, data_mc.payload)
            elif (player.pending() <= 0):
                print('End of MC file and all packets consumed')
                break

        ts_mc = player.get_timestamp()

        # Get corresponding PV frame
        if (ts_mc > 0):
            status, data_pv = rd_pv.get_next_packet(ts_mc, hl2ss_mx.TimePreference.PREFER_PAST)
            if (status is None):
                # End of file
                print('End of PV file')
                break
            if (status == hl2ss_mx.Status.OK):
                # Reader packets have the same structure as receiver packets
                print(f'Frame captured at {data_pv.timestamp}')
                print(f'Focal length: {data_pv.payload.focal_length}')
                print(f'Principal point: {data_pv.payload.principal_point}')
                print(f'Exposure Time: {data_pv.payload.exposure_time}')
                print(f'Exposure Compensation: {data_pv.payload.exposure_compensation}')
                print(f'Lens Position (Focus): {data_pv.payload.lens_position}')
                print(f'Focus State: {data_pv.payload.focus_state}')
                print(f'ISO Speed: {data_pv.payload.iso_speed}')
                print(f'White Balance: {data_pv.payload.white_balance}')
                print(f'ISO Gains: {data_pv.payload.iso_gains}')
                print(f'White Balance Gains: {data_pv.payload.white_balance_gains}')
                print(f'Resolution {data_pv.payload.resolution}')
                print(f'Pose')
                print(data_pv.pose)

                cv2.imshow('Video', data_pv.payload.image)

    # Cleanup -----------------------------------------------------------------
    player.close()
    rd_mc.close()
    rd_pv.close()

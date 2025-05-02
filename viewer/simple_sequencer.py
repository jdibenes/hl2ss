#------------------------------------------------------------------------------
# Sequencer example. The sequencer associates data from two or more files.
# Here, it is used to pair RM VLC LEFTFRONT and RM VLC RIGHTFRONT frames with 
# PV frames, all previously recorded using simple recorder.
#------------------------------------------------------------------------------

import os
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_io
import hl2ss_mx

# Settings --------------------------------------------------------------------

# Directory containing the recorded data
folder = './data/DS-2025-04-16-17-45-07/'

#------------------------------------------------------------------------------

# Create readers --------------------------------------------------------------
rd_pv = hl2ss_io.create_rd(os.path.join(folder, f'{hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)}.bin'), hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')
rd_lf = hl2ss_io.sequencer(hl2ss_io.create_rd(os.path.join(folder, f'{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_LEFTFRONT)}.bin'), hl2ss.ChunkSize.SINGLE_TRANSFER, True))
rd_rf = hl2ss_io.sequencer(hl2ss_io.create_rd(os.path.join(folder, f'{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_RIGHTFRONT)}.bin'), hl2ss.ChunkSize.SINGLE_TRANSFER, True))

# Open readers ----------------------------------------------------------------
rd_pv.open()
rd_lf.open()
rd_rf.open()

cv2.namedWindow('PV')
cv2.namedWindow('VLF')
cv2.namedWindow('VRF')

status_lf = hl2ss_mx.Status.DISCARDED
status_rf = hl2ss_mx.Status.DISCARDED

max_positive_offset_lf = 0
min_negative_offset_lf = 0

max_positive_offset_rf = 0
min_negative_offset_rf = 0

# Main loop -------------------------------------------------------------------
while ((cv2.waitKey(1) & 0xFF) != 27):
    # Get PV frame ------------------------------------------------------------
    data_pv = rd_pv.get_next_packet()
    if (data_pv is None):
        print('End of PV file')
        break

    # Find RM VLC frames corresponding to the current PV frame ----------------
    # Get nearest (in time) lf frame
    if (status_lf is not None):
        prev_lf = status_lf
        status_lf, data_lf = rd_lf.get_next_packet(data_pv.timestamp)
        if (status_lf != hl2ss_mx.Status.OK):
            data_lf = None
        if (status_lf is None):
            print('End of VLF file')
        elif ((prev_lf != hl2ss_mx.Status.OK) and (status_lf == hl2ss_mx.Status.OK)):
            print('VLF-PV in sync')
        elif ((prev_lf == hl2ss_mx.Status.OK) and (status_lf != hl2ss_mx.Status.OK)):
            print('VLF-PV out of sync')
    # Get nearest (in time) rf frame
    if (status_rf is not None):
        prev_rf = status_rf
        status_rf, data_rf = rd_rf.get_next_packet(data_pv.timestamp)
        if (status_rf != hl2ss_mx.Status.OK):
            data_rf = None
        if (status_rf is None):
            print('End of VRF file')
        elif ((prev_rf != hl2ss_mx.Status.OK) and (status_rf == hl2ss_mx.Status.OK)):
            print('VRF-PV in sync')
        elif ((prev_rf == hl2ss_mx.Status.OK) and (status_rf != hl2ss_mx.Status.OK)):
            print('VRF-PV out of sync')

    # Display frames ----------------------------------------------------------
    if (data_lf is not None):
        cv2.imshow('VLF', data_lf.payload.image)
        offset_lf = (data_lf.timestamp - data_pv.timestamp) / hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS
        if (offset_lf < min_negative_offset_lf):
            min_negative_offset_lf = offset_lf
        if (offset_lf > max_positive_offset_lf):
            max_positive_offset_lf = offset_lf
    if (data_rf is not None):
        cv2.imshow('VRF', data_rf.payload.image)
        offset_rf = (data_rf.timestamp - data_pv.timestamp) / hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS
        if (offset_rf < min_negative_offset_rf):
            min_negative_offset_rf = offset_rf
        if (offset_rf > max_positive_offset_rf):
            max_positive_offset_rf = offset_rf

    cv2.imshow('PV', data_pv.payload.image)

print(f'VLF-PV delta time: {[min_negative_offset_lf, max_positive_offset_lf]} seconds')
print(f'VRF-PV delta time: {[min_negative_offset_rf, max_positive_offset_rf]} seconds')

# Close readers ---------------------------------------------------------------
rd_pv.close()
rd_lf.close()
rd_rf.close()

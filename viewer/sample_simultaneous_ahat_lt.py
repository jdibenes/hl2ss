#------------------------------------------------------------------------------
# Experimental simultaneous RM Depth AHAT and RM Depth Long Throw.
# Press esc to stop.
#------------------------------------------------------------------------------

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Start streams -----------------------------------------------------------
    sink_lt = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    sink_ht = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_ahat(host, hl2ss.StreamPort.RM_DEPTH_AHAT))

    # Without this delay, the depth streams might crash and require rebooting 
    # the HoloLens to fix
    sink_lt.open()
    print('Waiting for sink_lt first frame')
    while (sink_lt.get_most_recent_frame()[1] is None):
        pass
    sink_ht.open()

    cv2.namedWindow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_AHAT) + '-depth')
    cv2.namedWindow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_AHAT) + '-ab')
    cv2.namedWindow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW) + '-depth')
    cv2.namedWindow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW) + '-ab')

    # Main loop ---------------------------------------------------------------
    while ((cv2.waitKey(1) & 0xFF) != 27):
        _, data_ht = sink_ht.get_most_recent_frame()
        _, data_lt = sink_lt.get_most_recent_frame()

        if (data_ht is not None):
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_AHAT) + '-depth', hl2ss_3dcv.rm_depth_colormap(data_ht.payload.depth, 1056)) # Scaled for visibility
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_AHAT) + '-ab', hl2ss_3dcv.rm_ab_normalize(data_ht.payload.ab)) # Scaled for visibility

        if (data_lt is not None):
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW) + '-depth', hl2ss_3dcv.rm_depth_colormap(data_lt.payload.depth, 7500)) # Scaled for visibility
            cv2.imshow(hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW) + '-ab', hl2ss_3dcv.rm_ab_normalize(data_lt.payload.ab)) # Scaled for visibility

    # Stop streams ------------------------------------------------------------
    sink_ht.close()
    sink_lt.close()

#------------------------------------------------------------------------------
# Example for streaming RGB and Depth frames from a RealSense camera connected
# to the HoloLens 2 USB-C port.
# Press ESC to stop.
#------------------------------------------------------------------------------

#check
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Camera groups
ev_group_index = 2
ez_group_index = 0

# Camera parameters (RealSense D435i)
ev_width = 1280
ev_height = 720
ev_fps = 30
ez_media_index = 29 # "29":{"Width":1280,"Height":720,"FrameRate":30,"Subtype":"Unknown"},
max_depth = 4095

#------------------------------------------------------------------------------

if __name__ == '__main__':
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, global_opacity=ev_group_index, output_width=0, output_height=0)
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH, global_opacity=ez_group_index, output_width=0, output_height=0)

    sink_ev = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, width=ev_width, height=ev_height, framerate=ev_fps))
    sink_ez = hl2ss_mp.stream(hl2ss_lnm.rx_extended_depth(host, hl2ss.StreamPort.EXTENDED_DEPTH, media_index=ez_media_index))

    sink_ev.open()
    sink_ez.open()

    cv2.namedWindow('RGB')
    cv2.namedWindow('Depth')

    while ((cv2.waitKey(1) & 0xFF) != 27):
        # Artificial delay of 6 frames to simplify RGB<->Depth frame pairing
        # might fail due to network lag
        _, _, data_ev = sink_ev.get_buffered_frame(-6)
        if (data_ev is None):
            continue

        _, data_ez = sink_ez.get_nearest(data_ev.timestamp)
        if (data_ez is None):
            continue

        if (data_ev.timestamp != data_ez.timestamp):
            ts_delta = (data_ev.timestamp-data_ez.timestamp) / hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS
            print(f'timestamp mismatch {data_ev.timestamp}-{data_ez.timestamp} delta {ts_delta} seconds ({ts_delta * ev_fps} RGB frames)')

        cv2.imshow('RGB', data_ev.payload.image)
        cv2.imshow('Depth', hl2ss_3dcv.rm_depth_colormap(data_ez.payload.depth, max_depth))

    sink_ez.close()
    sink_ev.close()

    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_DEPTH)
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO)

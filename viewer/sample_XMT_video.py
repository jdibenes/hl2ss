
import numpy as np
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_mt


# Settings --------------------------------------------------------------------

host = '192.168.1.7'
buffer_size = 512

ports = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    hl2ss.StreamPort.RM_DEPTH_AHAT,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    #hl2ss.StreamPort.EXTENDED_VIDEO,
]

# Camera selection for EV
# realsense
group_index = 2
source_index = 0
profile_index = 0

#------------------------------------------------------------------------------

rx = {
    hl2ss.StreamPort.RM_VLC_LEFTFRONT     : hl2ss_mt.rx_decoded_rm_vlc,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT      : hl2ss_mt.rx_decoded_rm_vlc,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : hl2ss_mt.rx_decoded_rm_vlc,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : hl2ss_mt.rx_decoded_rm_vlc,
    hl2ss.StreamPort.RM_DEPTH_AHAT        : hl2ss_mt.rx_decoded_rm_depth_ahat,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : hl2ss_mt.rx_decoded_rm_depth_longthrow,
    hl2ss.StreamPort.PERSONAL_VIDEO       : hl2ss_mt.rx_decoded_pv,
    hl2ss.StreamPort.EXTENDED_VIDEO       : hl2ss_mt.rx_decoded_pv
}

def create_default_rx(host, port, buffer_size):
    configuration = hl2ss_mt.create_configuration(port)
    configuration['width'] = 1280
    configuration['height'] = 720
    configuration['framerate'] = 30
    return rx[port](host, port, buffer_size, configuration)

def display_rm_vlc(data):
    return data.payload.image

def display_rm_depth(data):
    return np.hstack((data.payload.depth / np.max(data.payload.depth), data.payload.ab / np.max(data.payload.ab)))

def display_pv(data):
    return data.payload.image

display = {
    hl2ss.StreamPort.RM_VLC_LEFTFRONT     : display_rm_vlc,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT      : display_rm_vlc,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : display_rm_vlc,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : display_rm_vlc,
    hl2ss.StreamPort.RM_DEPTH_AHAT        : display_rm_depth,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : display_rm_depth,
    hl2ss.StreamPort.PERSONAL_VIDEO       : display_pv,
    hl2ss.StreamPort.EXTENDED_VIDEO       : display_pv
}

streams = { port : create_default_rx(host, port, buffer_size) for port in ports }

if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

if (hl2ss.StreamPort.EXTENDED_VIDEO in ports):
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO, global_opacity=group_index, output_width=source_index, output_height=profile_index)

for stream in streams.items():
    cv2.namedWindow(hl2ss.get_port_name(stream[0]))
    stream[1].open()

while (True):
    for stream in streams.items():
        data = stream[1].get_by_index(-1)
        if (data.status == 0):
            cv2.imshow(hl2ss.get_port_name(stream[0]), display[stream[0]](data))
    if ((cv2.waitKey(1) & 0xFF) == 27): # wait some time between get_by_[...] calls to not starve the internal writer
        break

for stream in streams.items():
    stream[1].close()

if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

if (hl2ss.StreamPort.EXTENDED_VIDEO in ports):
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.EXTENDED_VIDEO)

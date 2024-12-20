
import struct
import hl2ss


#------------------------------------------------------------------------------
# Default Settings
#------------------------------------------------------------------------------

def get_video_codec_default_factor(profile):
    name = hl2ss.get_video_codec_name(profile)
    return 1/75 if (name == 'h264') else 1/150 if (name == 'hevc') else 1.0


def get_video_codec_default_gop_size(framerate, divisor, profile):
    return max([1, framerate if (profile != hl2ss.VideoProfile.RAW) else 1])


def get_video_codec_bitrate(width, height, framerate, divisor, factor):
    return int(width*height*framerate*12*factor)


def get_video_codec_default_bitrate(width, height, framerate, divisor, profile):
    return get_video_codec_bitrate(width, height, framerate, divisor, get_video_codec_default_factor(profile))


def get_video_codec_default_options(width, height, framerate, divisor, profile):
    options = dict()
    options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = get_video_codec_default_gop_size(framerate, divisor, profile)
    return options


#------------------------------------------------------------------------------
# Stream Sync Period
#------------------------------------------------------------------------------

def get_sync_frame_stamp(frame_stamp, sync_period):
    return frame_stamp + ((sync_period - (frame_stamp % sync_period)) % sync_period)


def get_sync_period(rx):
    if (rx.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return 1
    if (rx.port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return 1
    if (rx.port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return 1
    if (rx.port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return 1
    if (rx.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.MICROPHONE):
        return 1
    if (rx.port == hl2ss.StreamPort.SPATIAL_INPUT):
        return 1
    if (rx.port == hl2ss.StreamPort.EXTENDED_EYE_TRACKER):
        return 1
    if (rx.port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return 1
    if (rx.port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.EXTENDED_DEPTH):
        return 1


#------------------------------------------------------------------------------
# Control
#------------------------------------------------------------------------------

def start_subsystem_pv(host, port, enable_mrc=False, hologram_composition=True, recording_indicator=False, video_stabilization=False, blank_protected=False, show_mesh=False, shared=False, global_opacity=0.9, output_width=0.0, output_height=0.0, video_stabilization_length=0, hologram_perspective=hl2ss.HologramPerspective.PV):
    hl2ss.start_subsystem_pv(host, port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective)


def stop_subsystem_pv(host, port):
    hl2ss.stop_subsystem_pv(host, port)


#------------------------------------------------------------------------------
# Modes 0, 1
#------------------------------------------------------------------------------

def rx_rm_vlc(host, port, chunk=hl2ss.ChunkSize.RM_VLC, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile=hl2ss.VideoProfile.H265_MAIN, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile)

    if (options is None):
        options = get_video_codec_default_options(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile)
        options[hl2ss.H26xEncoderProperty.HL2SSAPI_VLCHostTicksOffsetExposure] = struct.unpack('<Q', struct.pack('<d', 0.0))[0]
        options[hl2ss.H26xEncoderProperty.HL2SSAPI_VLCHostTicksOffsetConstant] = struct.unpack('<Q', struct.pack('<q', -125000))[0]
    else:
        options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = options.get(hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize, get_video_codec_default_gop_size(hl2ss.Parameters_RM_VLC.FPS, divisor, profile))
    
    return hl2ss.rx_decoded_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options) if (decoded) else hl2ss.rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options)


def rx_rm_depth_ahat(host, port, chunk=hl2ss.ChunkSize.RM_DEPTH_AHAT, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile_z=hl2ss.DepthProfile.SAME, profile_ab=hl2ss.VideoProfile.H265_MAIN, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab) * (16 if (profile_z == hl2ss.DepthProfile.SAME) else 1)

    if (options is None):
        options = get_video_codec_default_options(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab)
    else:
        options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = options.get(hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize, get_video_codec_default_gop_size(hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab))
    
    return hl2ss.rx_decoded_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options) if (decoded) else hl2ss.rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)


def rx_rm_depth_longthrow(host, port, chunk=hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode=hl2ss.StreamMode.MODE_1, divisor=1, png_filter=hl2ss.PNGFilterMode.PAETH, decoded=True):
    return hl2ss.rx_decoded_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter) if (decoded) else hl2ss.rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter)


def rx_rm_imu(host, port, chunk=hl2ss.ChunkSize.RM_IMU, mode=hl2ss.StreamMode.MODE_1):
    return hl2ss.rx_rm_imu(host, port, chunk, mode)


def rx_pv(host, port, chunk=hl2ss.ChunkSize.PERSONAL_VIDEO, mode=hl2ss.StreamMode.MODE_1, width=1920, height=1080, framerate=30, divisor=1, profile=hl2ss.VideoProfile.H265_MAIN, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded_format='bgr24'):
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(width, height, framerate, divisor, profile)

    if (options is None):
        options = get_video_codec_default_options(width, height, framerate, divisor, profile)
    else:
        options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = options.get(hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize, get_video_codec_default_gop_size(framerate, divisor, profile))
    
    return hl2ss.rx_decoded_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options, decoded_format) if (decoded_format) else hl2ss.rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)


def rx_microphone(host, port, chunk=hl2ss.ChunkSize.MICROPHONE, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2, decoded=True):
    return hl2ss.rx_decoded_microphone(host, port, chunk, profile, level) if (decoded) else hl2ss.rx_microphone(host, port, chunk, profile, level)


def rx_si(host, port, chunk=hl2ss.ChunkSize.SPATIAL_INPUT):
    return hl2ss.rx_si(host, port, chunk)


def rx_eet(host, port, chunk=hl2ss.ChunkSize.EXTENDED_EYE_TRACKER, fps=30):
    return hl2ss.rx_eet(host, port, chunk, fps)


def rx_extended_audio(host, port, chunk=hl2ss.ChunkSize.EXTENDED_AUDIO, mixer_mode=hl2ss.MixerMode.BOTH, loopback_gain=1.0, microphone_gain=1.0, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2, decoded=True):
    return hl2ss.rx_decoded_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level) if (decoded) else hl2ss.rx_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level)


def rx_extended_depth(host, port, chunk=hl2ss.ChunkSize.EXTENDED_DEPTH, mode=hl2ss.StreamMode.MODE_0, divisor=1, profile_z=hl2ss.DepthProfile.ZDEPTH, media_index=0xFFFFFFFF, stride_mask=0x3F):
    options = dict()
    options[hl2ss.H26xEncoderProperty.HL2SSAPI_VideoMediaIndex] = media_index
    options[hl2ss.H26xEncoderProperty.HL2SSAPI_VideoStrideMask] = stride_mask

    return hl2ss.rx_decoded_extended_depth(host, port, chunk, mode, divisor, profile_z, options)


#------------------------------------------------------------------------------
# Mode 2
#------------------------------------------------------------------------------

def download_calibration_rm_vlc(host, port):
    return hl2ss.download_calibration_rm_vlc(host, port)


def download_calibration_rm_depth_ahat(host, port):
    return hl2ss.download_calibration_rm_depth_ahat(host, port)


def download_calibration_rm_depth_longthrow(host, port):
    return hl2ss.download_calibration_rm_depth_longthrow(host, port)


def download_calibration_rm_imu(host, port):
    return hl2ss.download_calibration_rm_imu(host, port)


def download_calibration_pv(host, port, width, height, framerate):
    return hl2ss.download_calibration_pv(host, port, width, height, framerate)


def download_devicelist_extended_audio(host, port):
    return hl2ss.download_devicelist_extended_audio(host, port)


def download_devicelist_extended_video(host, port):
    return hl2ss.download_devicelist_extended_video(host, port)


#------------------------------------------------------------------------------
# IPC
#------------------------------------------------------------------------------

def ipc_rc(host, port):
    return hl2ss.ipc_rc(host, port)


def ipc_sm(host, port):
    return hl2ss.ipc_sm(host, port)


def ipc_su(host, port):
    return hl2ss.ipc_su(host, port)


def ipc_vi(host, port):
    return hl2ss.ipc_vi(host, port)


def ipc_umq(host, port):
    return hl2ss.ipc_umq(host, port)


def ipc_gmq(host, port):
    return hl2ss.ipc_gmq(host, port)


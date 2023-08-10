
import hl2ss


#------------------------------------------------------------------------------
# Default Settings
#------------------------------------------------------------------------------

def get_video_codec_default_factor(profile):
    name = hl2ss.get_video_codec_name(profile)
    return 1/50 if (name == 'h264') else 1/100 if (name == 'hevc') else 1.0


def get_video_codec_default_gop_size(framerate, divisor):
    return max([1, framerate])


def get_video_codec_bitrate(width, height, framerate, divisor, factor):
    return int(width*height*(framerate/divisor)*12*factor)


def get_video_codec_default_bitrate(width, height, framerate, divisor, profile):
    return get_video_codec_bitrate(width, height, framerate, divisor, get_video_codec_default_factor(profile))


def get_video_codec_default_options(width, height, framerate, divisor, profile):
    options = dict()
    options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = get_video_codec_default_gop_size(framerate, divisor) if (profile != hl2ss.VideoProfile.RAW) else 1
    return options


#------------------------------------------------------------------------------
# Receivers
#------------------------------------------------------------------------------

def rx_rm_vlc(host, port, chunk=hl2ss.ChunkSize.RM_VLC, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile=hl2ss.VideoProfile.H264_HIGH, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile)

    if (options is None):
        options = get_video_codec_default_options(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile)
    
    return hl2ss.rx_decoded_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options) if (decoded) else hl2ss.rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options)


def rx_rm_depth_ahat(host, port, chunk=hl2ss.ChunkSize.RM_DEPTH_AHAT, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile_z=hl2ss.DepthProfile.SAME, profile_ab=hl2ss.VideoProfile.H264_HIGH, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab) * (4 if ((profile_z == hl2ss.DepthProfile.SAME) and (profile_ab != hl2ss.VideoProfile.RAW)) else 1)

    if (options is None):
        options = get_video_codec_default_options(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile_ab)
    
    return hl2ss.rx_decoded_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options) if (decoded) else hl2ss.rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)


def rx_rm_depth_longthrow(host, port, chunk=hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode=hl2ss.StreamMode.MODE_1, divisor=1, png_filter=hl2ss.PNGFilterMode.PAETH, decoded=True):
    return hl2ss.rx_decoded_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter) if (decoded) else hl2ss.rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter)


def rx_rm_imu(host, port, chunk=hl2ss.ChunkSize.RM_IMU, mode=hl2ss.StreamMode.MODE_1):
    return hl2ss.rx_rm_imu(host, port, chunk, mode)


def rx_pv(host, port, chunk=hl2ss.ChunkSize.PERSONAL_VIDEO, mode=hl2ss.StreamMode.MODE_1, width=1920, height=1080, framerate=30, divisor=1, profile=hl2ss.VideoProfile.H264_HIGH, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded_format='bgr24'):
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(width, height, framerate, divisor, profile)

    if (options is None):
        options = get_video_codec_default_options(width, height, framerate, divisor, profile)
    
    return hl2ss.rx_decoded_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options, decoded_format) if (decoded_format) else hl2ss.rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)


def rx_microphone(host, port, chunk=hl2ss.ChunkSize.MICROPHONE, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2, decoded=True):
    return hl2ss.rx_decoded_microphone(host, port, chunk, profile, level) if (decoded) else hl2ss.rx_microphone(host, port, chunk, profile, level)


def rx_si(host, port, chunk=hl2ss.ChunkSize.SPATIAL_INPUT):
    return hl2ss.rx_si(host, port, chunk)


def rx_eet(host, port, chunk=hl2ss.ChunkSize.EXTENDED_EYE_TRACKER, fps=30):
    return hl2ss.rx_eet(host, port, chunk, fps)


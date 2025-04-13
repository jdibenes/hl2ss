
import struct
import hl2ss
import hl2ss_dp


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


def create_configuration_for_dp_mrc(pv=True, holo=False, mic=True, loopback=False, RenderFromCamera=True, vstab=False, vstabbuffer=15):
    return hl2ss_dp.create_configuration_for_mrc(pv, holo, mic, loopback, RenderFromCamera, vstab, vstabbuffer)


def create_sockopt(settimeout=None, tcp_nodelay=1):
    sockopt = {
        'settimeout' : settimeout,
        'setsockopt.IPPROTO_TCP.TCP_NODELAY' : tcp_nodelay,
    }
    return sockopt


#------------------------------------------------------------------------------
# Control
#------------------------------------------------------------------------------

def start_subsystem_pv(host, port, sockopt=None, enable_mrc=False, hologram_composition=True, recording_indicator=False, video_stabilization=False, blank_protected=False, show_mesh=False, shared=False, global_opacity=0.9, output_width=0.0, output_height=0.0, video_stabilization_length=0, hologram_perspective=hl2ss.HologramPerspective.PV):
    if (sockopt is None):
        sockopt = create_sockopt()

    hl2ss.start_subsystem_pv(host, port, sockopt, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective)


def stop_subsystem_pv(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    hl2ss.stop_subsystem_pv(host, port, sockopt)


#------------------------------------------------------------------------------
# Modes 0, 1
#------------------------------------------------------------------------------

def rx_rm_vlc(host, port, sockopt=None, chunk=hl2ss.ChunkSize.RM_VLC, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile=hl2ss.VideoProfile.H265_MAIN, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile)

    if (options is None):
        options = get_video_codec_default_options(hl2ss.Parameters_RM_VLC.WIDTH, hl2ss.Parameters_RM_VLC.HEIGHT, hl2ss.Parameters_RM_VLC.FPS, divisor, profile)
        options[hl2ss.H26xEncoderProperty.HL2SSAPI_VLCHostTicksOffsetExposure] = struct.unpack('<Q', struct.pack('<d', 0.0))[0]
        options[hl2ss.H26xEncoderProperty.HL2SSAPI_VLCHostTicksOffsetConstant] = struct.unpack('<Q', struct.pack('<q', -125000))[0]
    else:
        options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = options.get(hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize, get_video_codec_default_gop_size(hl2ss.Parameters_RM_VLC.FPS, divisor, profile))
    
    return hl2ss.rx_decoded_rm_vlc(host, port, sockopt, chunk, mode, divisor, profile, level, bitrate, options) if (decoded) else hl2ss.rx_rm_vlc(host, port, sockopt, chunk, mode, divisor, profile, level, bitrate, options)


def rx_rm_depth_ahat(host, port, sockopt=None, chunk=hl2ss.ChunkSize.RM_DEPTH_AHAT, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile_z=hl2ss.DepthProfile.SAME, profile_ab=hl2ss.VideoProfile.H265_MAIN, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab) * (16 if (profile_z == hl2ss.DepthProfile.SAME) else 1)

    if (options is None):
        options = get_video_codec_default_options(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab)
    else:
        options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = options.get(hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize, get_video_codec_default_gop_size(hl2ss.Parameters_RM_DEPTH_AHAT.FPS, divisor, profile_ab))
    
    return hl2ss.rx_decoded_rm_depth_ahat(host, port, sockopt, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options) if (decoded) else hl2ss.rx_rm_depth_ahat(host, port, sockopt, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)


def rx_rm_depth_longthrow(host, port, sockopt=None, chunk=hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode=hl2ss.StreamMode.MODE_1, divisor=1, png_filter=hl2ss.PNGFilterMode.PAETH, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.rx_decoded_rm_depth_longthrow(host, port, sockopt, chunk, mode, divisor, png_filter) if (decoded) else hl2ss.rx_rm_depth_longthrow(host, port, sockopt, chunk, mode, divisor, png_filter)


def rx_rm_imu(host, port, sockopt=None, chunk=hl2ss.ChunkSize.RM_IMU, mode=hl2ss.StreamMode.MODE_1, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.rx_decoded_rm_imu(host, port, sockopt, chunk, mode) if (decoded) else hl2ss.rx_rm_imu(host, port, sockopt, chunk, mode)


def rx_pv(host, port, sockopt=None, chunk=hl2ss.ChunkSize.PERSONAL_VIDEO, mode=hl2ss.StreamMode.MODE_1, width=1920, height=1080, framerate=30, divisor=1, profile=hl2ss.VideoProfile.H265_MAIN, level=hl2ss.H26xLevel.DEFAULT, bitrate=None, options=None, decoded_format='bgr24'):
    if (sockopt is None):
        sockopt = create_sockopt()
    
    if (bitrate is None):
        bitrate = get_video_codec_default_bitrate(width, height, framerate, divisor, profile)

    if (options is None):
        options = get_video_codec_default_options(width, height, framerate, divisor, profile)
    else:
        options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize] = options.get(hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize, get_video_codec_default_gop_size(framerate, divisor, profile))
    
    return hl2ss.rx_decoded_pv(host, port, sockopt, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options, decoded_format) if (decoded_format) else hl2ss.rx_pv(host, port, sockopt, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)


def rx_microphone(host, port, sockopt=None, chunk=hl2ss.ChunkSize.MICROPHONE, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.rx_decoded_microphone(host, port, sockopt, chunk, profile, level) if (decoded) else hl2ss.rx_microphone(host, port, sockopt, chunk, profile, level)


def rx_si(host, port, sockopt=None, chunk=hl2ss.ChunkSize.SPATIAL_INPUT, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.rx_decoded_si(host, port, sockopt, chunk) if (decoded) else hl2ss.rx_si(host, port, sockopt, chunk)


def rx_eet(host, port, sockopt=None, chunk=hl2ss.ChunkSize.EXTENDED_EYE_TRACKER, fps=30, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.rx_decoded_eet(host, port, sockopt, chunk, fps) if (decoded) else hl2ss.rx_eet(host, port, sockopt, chunk, fps)


def rx_extended_audio(host, port, sockopt=None, chunk=hl2ss.ChunkSize.EXTENDED_AUDIO, mixer_mode=hl2ss.MixerMode.BOTH, loopback_gain=1.0, microphone_gain=1.0, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()
    
    return hl2ss.rx_decoded_extended_audio(host, port, sockopt, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level) if (decoded) else hl2ss.rx_extended_audio(host, port, sockopt, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level)


def rx_extended_depth(host, port, sockopt=None, chunk=hl2ss.ChunkSize.EXTENDED_DEPTH, mode=hl2ss.StreamMode.MODE_1, divisor=1, profile_z=hl2ss.DepthProfile.ZDEPTH, media_index=0xFFFFFFFF, stride_mask=0x3F, decoded=True):
    if (sockopt is None):
        sockopt = create_sockopt()
    
    options = dict()
    options[hl2ss.H26xEncoderProperty.HL2SSAPI_VideoMediaIndex] = media_index
    options[hl2ss.H26xEncoderProperty.HL2SSAPI_VideoStrideMask] = stride_mask

    return hl2ss.rx_decoded_extended_depth(host, port, sockopt, chunk, mode, divisor, profile_z, options) if (decoded) else hl2ss.rx_extended_depth(host, port, sockopt, chunk, mode, divisor, profile_z, options)


def rx_dp_mrc(host, port, user, password, chunk=hl2ss_dp.ChunkSize.MRC, configuration=None, decoded_format='bgr24'):
    if (configuration is None):
        configuration = create_configuration_for_dp_mrc()

    return hl2ss_dp.rx_decoded_mrc(host, port, user, password, chunk, configuration, decoded_format) if (decoded_format) else hl2ss_dp.rx_mrc(host, port, user, password, chunk, configuration)


#------------------------------------------------------------------------------
# Mode 2
#------------------------------------------------------------------------------

def download_calibration_rm_vlc(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_calibration_rm_vlc(host, port, sockopt)


def download_calibration_rm_depth_ahat(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_calibration_rm_depth_ahat(host, port, sockopt)


def download_calibration_rm_depth_longthrow(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_calibration_rm_depth_longthrow(host, port, sockopt)


def download_calibration_rm_imu(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_calibration_rm_imu(host, port, sockopt)


def download_calibration_pv(host, port, sockopt=None, width=1920, height=1080, framerate=30):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_calibration_pv(host, port, sockopt, width, height, framerate)


def download_devicelist_extended_audio(host, port, sockopt=None, profile=hl2ss.AudioProfile.AAC_24000, level=hl2ss.AACLevel.L2):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_devicelist_extended_audio(host, port, sockopt, profile, level)


def download_devicelist_extended_video(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.download_devicelist_extended_video(host, port, sockopt)


#------------------------------------------------------------------------------
# IPC
#------------------------------------------------------------------------------

def ipc_rc(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.ipc_rc(host, port, sockopt)


def ipc_sm(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.ipc_sm(host, port, sockopt)


def ipc_su(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.ipc_su(host, port, sockopt)


def ipc_vi(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.ipc_vi(host, port, sockopt)


def ipc_umq(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.ipc_umq(host, port, sockopt)


def ipc_gmq(host, port, sockopt=None):
    if (sockopt is None):
        sockopt = create_sockopt()

    return hl2ss.ipc_gmq(host, port, sockopt)



import struct
import types
import hl2ss


_MAGIC = 'HL2SSV23'


#------------------------------------------------------------------------------
# File Writer
#------------------------------------------------------------------------------

class _writer:
    def open(self, filename):
        self._file = open(filename, 'wb')

    def put(self, data):
        self._file.write(data)

    def write(self, packet):
        self._file.write(hl2ss.pack_packet(packet))

    def close(self):
        self._file.close()


#------------------------------------------------------------------------------
# Header Pack
#------------------------------------------------------------------------------

def _create_header(port, user):
    return struct.pack(f'<{len(_MAGIC)}sHI', _MAGIC.encode(), port, len(user)) + user


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Store
#------------------------------------------------------------------------------

def _create_wr_rm_vlc(filename, port, mode, divisor, profile, level, bitrate, options, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_rm_vlc(mode, divisor, profile, level, bitrate, options))
    return w


def _create_wr_rm_depth_ahat(filename, port, mode, divisor, profile_z, profile_ab, level, bitrate, options, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_rm_depth_ahat(mode, divisor, profile_z, profile_ab, level, bitrate, options))
    return w


def _create_wr_rm_depth_longthrow(filename, port, mode, divisor, png_filter, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_rm_depth_longthrow(mode, divisor, png_filter))
    return w


def _create_wr_rm_imu(filename, port, mode, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_rm_imu(mode))
    return w


def _create_wr_pv(filename, port, mode, width, height, framerate, divisor, profile, level, bitrate, options, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_pv(mode, width, height, framerate, divisor, profile, level, bitrate, options))
    return w


def _create_wr_microphone(filename, port, profile, level, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_microphone(profile, level))
    return w


def _create_wr_si(filename, port, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    return w


def _create_wr_eet(filename, port, fps, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_eet(fps))
    return w


def _create_wr_extended_audio(filename, port, mixer_mode, loopback_gain, microphone_gain, profile, level, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_extended_audio(mixer_mode, loopback_gain, microphone_gain, profile, level))
    return w


def _create_wr_extended_depth(filename, port, mode, divisor, profile_z, options, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port, user))
    w.put(hl2ss._create_configuration_for_extended_depth(mode, divisor, profile_z, options))
    return w


#------------------------------------------------------------------------------
# Writer Wrappers
#------------------------------------------------------------------------------

class wr_rm_vlc(hl2ss._context_manager):
    def __init__(self, filename, port, mode, divisor, profile, level, bitrate, options, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.divisor = divisor
        self.profile = profile
        self.level = level
        self.bitrate = bitrate
        self.options = options
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_vlc(self.filename, self.port, self.mode, self.divisor, self.profile, self.level, self.bitrate, self.options, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_rm_depth_ahat(hl2ss._context_manager):
    def __init__(self, filename, port, mode, divisor, profile_z, profile_ab, level, bitrate, options, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.profile_ab = profile_ab
        self.level = level
        self.bitrate = bitrate
        self.options = options
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_depth_ahat(self.filename, self.port, self.mode, self.divisor, self.profile_z, self.profile_ab, self.level, self.bitrate, self.options, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_rm_depth_longthrow(hl2ss._context_manager):
    def __init__(self, filename, port, mode, divisor, png_filter, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.divisor = divisor
        self.png_filter = png_filter
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_depth_longthrow(self.filename, self.port, self.mode, self.divisor, self.png_filter, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_rm_imu(hl2ss._context_manager):
    def __init__(self, filename, port, mode, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_imu(self.filename, self.port, self.mode, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_pv(hl2ss._context_manager):
    def __init__(self, filename, port, mode, width, height, framerate, divisor, profile, level, bitrate, options, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.width = width
        self.height = height
        self.framerate = framerate
        self.divisor = divisor
        self.profile = profile
        self.level = level
        self.bitrate = bitrate
        self.options = options
        self.user = user

    def open(self):
        self._wr = _create_wr_pv(self.filename, self.port, self.mode, self.width, self.height, self.framerate, self.divisor, self.profile, self.level, self.bitrate, self.options, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_microphone(hl2ss._context_manager):
    def __init__(self, filename, port, profile, level, user):
        self.filename = filename
        self.port = port
        self.profile = profile
        self.level = level
        self.user = user
    
    def open(self):
        self._wr = _create_wr_microphone(self.filename, self.port, self.profile, self.level, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_si(hl2ss._context_manager):
    def __init__(self, filename, port, user):
        self.filename = filename
        self.port = port
        self.user = user

    def open(self):
        self._wr = _create_wr_si(self.filename, self.port, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_eet(hl2ss._context_manager):
    def __init__(self, filename, port, fps, user):
        self.filename = filename
        self.port = port
        self.fps = fps
        self.user = user

    def open(self):
        self._wr = _create_wr_eet(self.filename, self.port, self.fps, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_extended_audio(hl2ss._context_manager):
    def __init__(self, filename, port, mixer_mode, loopback_gain, microphone_gain, profile, level, user):
        self.filename = filename
        self.port = port
        self.mixer_mode = mixer_mode
        self.loopback_gain = loopback_gain
        self.microphone_gain = microphone_gain
        self.profile = profile
        self.level = level
        self.user = user

    def open(self):
        self._wr = _create_wr_extended_audio(self.filename, self.port, self.mixer_mode, self.loopback_gain, self.microphone_gain, self.profile, self.level, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_extended_depth(hl2ss._context_manager):
    def __init__(self, filename, port, mode, divisor, profile_z, options, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.divisor = divisor
        self.profile_z = profile_z
        self.options = options
        self.user = user

    def open(self):
        self._wr = _create_wr_extended_depth(self.filename, self.port, self.mode, self.divisor, self.profile_z, self.options, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


#------------------------------------------------------------------------------
# Writer From Receiver
#------------------------------------------------------------------------------

def _create_wr_from_rx_rm_vlc(filename, rx, user):
    return wr_rm_vlc(filename, rx.port, rx.mode, rx.divisor, rx.profile, rx.level, rx.bitrate, rx.options, user)


def _create_wr_from_rx_rm_depth_ahat(filename, rx, user):
    return wr_rm_depth_ahat(filename, rx.port, rx.mode, rx.divisor, rx.profile_z, rx.profile_ab, rx.level, rx.bitrate, rx.options, user)


def _create_wr_from_rx_rm_depth_longthrow(filename, rx, user):
    return wr_rm_depth_longthrow(filename, rx.port, rx.mode, rx.divisor, rx.png_filter, user)


def _create_wr_from_rx_rm_imu(filename, rx, user):
    return wr_rm_imu(filename, rx.port, rx.mode, user)


def _create_wr_from_rx_pv(filename, rx, user):
    return wr_pv(filename, rx.port, rx.mode, rx.width, rx.height, rx.framerate, rx.divisor, rx.profile, rx.level, rx.bitrate, rx.options, user)


def _create_wr_from_rx_microphone(filename, rx, user):
    return wr_microphone(filename, rx.port, rx.profile, rx.level, user)


def _create_wr_from_rx_si(filename, rx, user):
    return wr_si(filename, rx.port, user)


def _create_wr_from_rx_eet(filename, rx, user):
    return wr_eet(filename, rx.port, rx.fps, user)


def _create_wr_from_rx_extended_audio(filename, rx, user):
    return wr_extended_audio(filename, rx.port, rx.mixer_mode, rx.loopback_gain, rx.microphone_gain, rx.profile, rx.level, user)


def _create_wr_from_rx_extended_depth(filename, rx, user):
    return wr_extended_depth(filename, rx.port, rx.mode, rx.divisor, rx.profile_z, rx.options, user)


def create_wr_from_rx(filename, rx, user):
    if (rx.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return _create_wr_from_rx_rm_vlc(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return _create_wr_from_rx_rm_vlc(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return _create_wr_from_rx_rm_vlc(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return _create_wr_from_rx_rm_vlc(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return _create_wr_from_rx_rm_depth_ahat(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return _create_wr_from_rx_rm_depth_longthrow(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return _create_wr_from_rx_rm_imu(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return _create_wr_from_rx_rm_imu(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return _create_wr_from_rx_rm_imu(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return _create_wr_from_rx_pv(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.MICROPHONE):
        return _create_wr_from_rx_microphone(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.SPATIAL_INPUT):
        return _create_wr_from_rx_si(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.EXTENDED_EYE_TRACKER):
        return _create_wr_from_rx_eet(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return _create_wr_from_rx_extended_audio(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return _create_wr_from_rx_pv(filename, rx, user)
    if (rx.port == hl2ss.StreamPort.EXTENDED_DEPTH):
        return _create_wr_from_rx_extended_depth(filename, rx, user)


#------------------------------------------------------------------------------
# File Reader
#------------------------------------------------------------------------------

class _reader:
    def open(self, filename, chunk):
        self._file = open(filename, 'rb')
        self._chunk = chunk
        
    def get(self, format):
        return struct.unpack(format, self._file.read(struct.calcsize(format)))
    
    def get_header(self):
        return self.get(f'<{len(_MAGIC)}sH') + (self._file.read(self.get('<I')[0]),)
    
    def get_configuration_for_mode(self):
        return self.get('<B')
    
    def get_configuration_for_video_format(self):
        return self.get('<HHB')
    
    def get_configuration_for_video_divisor(self):
        return self.get('<B')
    
    def get_configuration_for_video_encoding(self):
        return self.get('<BBI')
    
    def get_configuration_for_depth_encoding(self):
        return self.get('<B')
    
    def get_configuration_for_audio_encoding(self):
        return self.get('<BB')
    
    def get_configuration_for_png_encoding(self):
        return self.get('<B')
    
    def get_configuration_for_h26x_encoding(self):
        count = self.get('<B')[0]
        vector = self.get(f'<{2*count}Q')
        return ({ vector[2*i] : vector[2*i+1] for i in range(0, count) },)
    
    def get_configuration_for_framerate(self):
        return self.get('<B')
    
    def get_configuration_for_mrc_audio(self):
        return self.get('<Iff')
    
    def get_configuration_for_rm_vlc(self):
        return self.get_configuration_for_mode() + self.get_configuration_for_video_divisor() + self.get_configuration_for_video_encoding() + self.get_configuration_for_h26x_encoding()
    
    def get_configuration_for_rm_depth_ahat(self):
        return self.get_configuration_for_mode() + self.get_configuration_for_video_divisor() + self.get_configuration_for_depth_encoding() + self.get_configuration_for_video_encoding() + self.get_configuration_for_h26x_encoding()

    def get_configuration_for_rm_depth_longthrow(self):
        return self.get_configuration_for_mode() + self.get_configuration_for_video_divisor() + self.get_configuration_for_png_encoding()
    
    def get_configuration_for_rm_imu(self):
        return self.get_configuration_for_mode()[0]
    
    def get_configuration_for_pv(self):
        return self.get_configuration_for_mode() + self.get_configuration_for_video_format() + self.get_configuration_for_video_divisor() + self.get_configuration_for_video_encoding() + self.get_configuration_for_h26x_encoding()
    
    def get_configuration_for_microphone(self):
        return self.get_configuration_for_audio_encoding()
    
    def get_configuration_for_eet(self):
        return self.get_configuration_for_framerate()[0]
    
    def get_configuration_for_extended_audio(self):
        return self.get_configuration_for_mrc_audio() + self.get_configuration_for_audio_encoding()

    def get_configuration_for_extended_depth(self):
        return self.get_configuration_for_mode() + self.get_configuration_for_video_divisor() + self.get_configuration_for_depth_encoding() + self.get_configuration_for_h26x_encoding()

    def begin(self, mode):
        self._unpacker = hl2ss._unpacker()
        self._unpacker.reset(mode)
        self._eof = False
        
    def get_next_packet(self):
        while (True):
            if (self._unpacker.unpack()):
                return self._unpacker.get()
            if (self._eof):
                return None
            chunk = self._file.read(self._chunk)
            self._eof = len(chunk) < self._chunk
            self._unpacker.extend(chunk)

    def close(self):
        self._file.close()


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Load
#------------------------------------------------------------------------------

def _create_rd(filename, chunk):
    rd = _reader()
    rd.open(filename, chunk)
    return (rd,) + rd.get_header()


#------------------------------------------------------------------------------
# Reader Wrapper
#------------------------------------------------------------------------------

class _rd(hl2ss._context_manager):
    def __load_rm_vlc(self):
        self.mode, self.divisor, self.profile, self.level, self.bitrate, self.options = self._rd.get_configuration_for_rm_vlc()
        self._rd.begin(self.mode)

    def __load_rm_depth_ahat(self):
        self.mode, self.divisor, self.profile_z, self.profile_ab, self.level, self.bitrate, self.options = self._rd.get_configuration_for_rm_depth_ahat()
        self._rd.begin(self.mode)

    def __load_rm_depth_longthrow(self):
        self.mode, self.divisor, self.png_filter = self._rd.get_configuration_for_rm_depth_longthrow()
        self._rd.begin(self.mode)

    def __load_rm_imu(self):
        self.mode = self._rd.get_configuration_for_rm_imu()
        self._rd.begin(self.mode)

    def __load_pv(self):
        self.mode, self.width, self.height, self.framerate, self.divisor, self.profile, self.level, self.bitrate, self.options = self._rd.get_configuration_for_pv()
        self._rd.begin(self.mode)

    def __load_microphone(self):
        self.profile, self.level = self._rd.get_configuration_for_microphone()
        self._rd.begin(hl2ss.StreamMode.MODE_0)

    def __load_si(self):
        self._rd.begin(hl2ss.StreamMode.MODE_0)

    def __load_eet(self):
        self.fps = self._rd.get_configuration_for_eet()
        self._rd.begin(hl2ss.StreamMode.MODE_1)

    def __load_extended_audio(self):
        self.mixer_mode, self.loopback_gain, self.microphone_gain, self.profile, self.level = self._rd.get_configuration_for_extended_audio()
        self._rd.begin(hl2ss.StreamMode.MODE_0)

    def __load_extended_depth(self):
        self.mode, self.divisor, self.profile_z, self.options = self._rd.get_configuration_for_extended_depth()

    __method_table = {
        hl2ss.StreamPort.RM_VLC_LEFTFRONT     : (__load_rm_vlc,),
        hl2ss.StreamPort.RM_VLC_LEFTLEFT      : (__load_rm_vlc,),
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : (__load_rm_vlc,),
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : (__load_rm_vlc,),
        hl2ss.StreamPort.RM_DEPTH_AHAT        : (__load_rm_depth_ahat,),
        hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : (__load_rm_depth_longthrow,),
        hl2ss.StreamPort.RM_IMU_ACCELEROMETER : (__load_rm_imu,),
        hl2ss.StreamPort.RM_IMU_GYROSCOPE     : (__load_rm_imu,),
        hl2ss.StreamPort.RM_IMU_MAGNETOMETER  : (__load_rm_imu,),
        hl2ss.StreamPort.PERSONAL_VIDEO       : (__load_pv,),
        hl2ss.StreamPort.MICROPHONE           : (__load_microphone,),
        hl2ss.StreamPort.SPATIAL_INPUT        : (__load_si,),
        hl2ss.StreamPort.EXTENDED_EYE_TRACKER : (__load_eet,),
        hl2ss.StreamPort.EXTENDED_AUDIO       : (__load_extended_audio,),
        hl2ss.StreamPort.EXTENDED_VIDEO       : (__load_pv,),
        hl2ss.StreamPort.EXTENDED_DEPTH       : (__load_extended_depth),
    }

    def __build(self):
        f = _rd.__method_table[self.port]
        self.__load = types.MethodType(f[0], self)
        
    def __init__(self, filename, chunk):
        self.filename = filename
        self.chunk = chunk

    def open(self):
        self._rd, self.magic, self.port, self.user = _create_rd(self.filename, self.chunk)
        self.__build()
        self.__load()
        
    def get_next_packet(self):
        return self._rd.get_next_packet()

    def close(self):
        self._rd.close()


#------------------------------------------------------------------------------
# Decoded Readers
#------------------------------------------------------------------------------

class _rd_decoded(_rd):
    def __set_codec_rm_vlc(self):
        self._codec = hl2ss.decode_rm_vlc(self.profile)

    def __set_codec_rm_depth_ahat(self):
        self._codec = hl2ss.decode_rm_depth_ahat(self.profile_z, self.profile_ab)

    def __set_codec_rm_depth_longthrow(self):
        pass

    def __set_codec_rm_imu(self):
        pass

    def __set_codec_pv(self):
        self._codec = hl2ss.decode_pv(self.profile)

    def __set_codec_microphone(self):
        self._codec = hl2ss.decode_microphone(self.profile, self.level)

    def __set_codec_si(self):
        pass

    def __set_codec_eet(self):
        pass

    def __set_codec_extended_audio(self):
        self._codec = hl2ss.decode_microphone(self.profile, None)

    def __set_codec_extended_depth(self):
        self._codec = hl2ss.decode_extended_depth(self.profile_z)

    def __create_codec_rm_vlc(self):
        self._codec.create()
        self.get_next_packet()

    def __create_codec_rm_depth_ahat(self):
        self._codec.create()
        self.get_next_packet()

    def __create_codec_rm_depth_longthrow(self):
        pass

    def __create_codec_rm_imu(self):
        pass

    def __create_codec_pv(self):
        self._codec.create(self.width, self.height)
        self.get_next_packet()

    def __create_codec_microphone(self):
        self._codec.create()

    def __create_codec_si(self):
        pass

    def __create_codec_eet(self):
        pass

    def __create_codec_extended_depth(self):
        self._codec.create()

    def __decode_rm_vlc(self, payload):
        payload = hl2ss.unpack_rm_vlc(payload)
        payload.image = self._codec.decode(payload.image)
        return payload
    
    def __decode_rm_depth_ahat(self, payload):
        return self._codec.decode(payload)
    
    def __decode_rm_depth_longthrow(self, payload):
        return hl2ss.decode_rm_depth_longthrow(payload)
    
    def __decode_rm_imu(self, payload):
        return payload
    
    def __decode_pv(self, payload):
        payload = hl2ss.unpack_pv(payload)
        payload.image = self._codec.decode(payload.image, self.format)
        return payload
    
    def __decode_microphone(self, payload):
        return self._codec.decode(payload)
    
    def __decode_si(self, payload):
        return payload
    
    def __decode_eet(self, payload):
        return payload
    
    def __decode_extended_depth(self, payload):
        return self._codec.decode(payload)
    
    __method_table = {
        hl2ss.StreamPort.RM_VLC_LEFTFRONT     : (__set_codec_rm_vlc,             __create_codec_rm_vlc,             __decode_rm_vlc),
        hl2ss.StreamPort.RM_VLC_LEFTLEFT      : (__set_codec_rm_vlc,             __create_codec_rm_vlc,             __decode_rm_vlc),
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : (__set_codec_rm_vlc,             __create_codec_rm_vlc,             __decode_rm_vlc),
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : (__set_codec_rm_vlc,             __create_codec_rm_vlc,             __decode_rm_vlc),
        hl2ss.StreamPort.RM_DEPTH_AHAT        : (__set_codec_rm_depth_ahat,      __create_codec_rm_depth_ahat,      __decode_rm_depth_ahat),
        hl2ss.StreamPort.RM_DEPTH_LONGTHROW   : (__set_codec_rm_depth_longthrow, __create_codec_rm_depth_longthrow, __decode_rm_depth_longthrow),
        hl2ss.StreamPort.RM_IMU_ACCELEROMETER : (__set_codec_rm_imu,             __create_codec_rm_imu,             __decode_rm_imu),
        hl2ss.StreamPort.RM_IMU_GYROSCOPE     : (__set_codec_rm_imu,             __create_codec_rm_imu,             __decode_rm_imu),
        hl2ss.StreamPort.RM_IMU_MAGNETOMETER  : (__set_codec_rm_imu,             __create_codec_rm_imu,             __decode_rm_imu),
        hl2ss.StreamPort.PERSONAL_VIDEO       : (__set_codec_pv,                 __create_codec_pv,                 __decode_pv),
        hl2ss.StreamPort.MICROPHONE           : (__set_codec_microphone,         __create_codec_microphone,         __decode_microphone),
        hl2ss.StreamPort.SPATIAL_INPUT        : (__set_codec_si,                 __create_codec_si,                 __decode_si),
        hl2ss.StreamPort.EXTENDED_EYE_TRACKER : (__set_codec_eet,                __create_codec_eet,                __decode_eet),
        hl2ss.StreamPort.EXTENDED_AUDIO       : (__set_codec_extended_audio,     __create_codec_microphone,         __decode_microphone),
        hl2ss.StreamPort.EXTENDED_VIDEO       : (__set_codec_pv,                 __create_codec_pv,                 __decode_pv),
        hl2ss.StreamPort.EXTENDED_DEPTH       : (__set_codec_extended_depth,     __create_codec_extended_depth,     __decode_extended_depth)
    }

    def __build(self):
        f = _rd_decoded.__method_table[self.port]
        self.__set_codec    = types.MethodType(f[0], self)
        self.__create_codec = types.MethodType(f[1], self)
        self.__decode       = types.MethodType(f[2], self)

    def __init__(self, filename, chunk, format):
        super().__init__(filename, chunk)
        self.format = format

    def open(self):
        super().open()
        self.__build()
        self.__set_codec()
        self.__create_codec()
        
    def get_next_packet(self):
        data = super().get_next_packet()
        if (data is not None):
            data.payload = self.__decode(data.payload)
        return data

    def close(self):
        super().close()


#------------------------------------------------------------------------------
# Create Reader
#------------------------------------------------------------------------------

def create_rd(filename, chunk, decoded):
    return _rd_decoded(filename, chunk, decoded) if (decoded) else _rd(filename, chunk)


#------------------------------------------------------------------------------
# Sequencer
#------------------------------------------------------------------------------

class sequencer:
    def __init__(self, filename, chunk, decoded):
        self.filename = filename
        self.chunk = chunk
        self.decoded = decoded

    def open(self):
        self._rd = create_rd(self.filename, self.chunk, self.decoded)
        self._rd.open()
        self._l = self._rd.get_next_packet()
        self._r = self._rd.get_next_packet()

    def get_next_packet(self, timestamp):
        if ((self._l is None) or (self._r is None)):
            return None
        if (timestamp < self._l.timestamp):
            return None
        while (timestamp > self._r.timestamp):
            self._l = self._r
            self._r = self._rd.get_next_packet()
            if (self._r is None):
                return None
        return self._l if ((timestamp - self._l.timestamp) < (self._r.timestamp - timestamp)) else self._r
    
    def close(self):
        self._rd.close()


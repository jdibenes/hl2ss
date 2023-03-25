
import multiprocessing as mp
import struct
import hl2ss
import hl2ss_mp


_MAGIC = 'X38HL2SS'


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

def _create_header(port):
    return struct.pack(f'<{len(_MAGIC)}sH', _MAGIC.encode(), port)


def _create_user(user):
    return struct.pack('<I', len(user)) + user


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Store
#------------------------------------------------------------------------------

def _create_wr_rm_vlc(filename, port, mode, profile, bitrate, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(hl2ss._create_configuration_for_rm_vlc(mode, profile, bitrate))
    w.put(_create_user(user))
    return w


def _create_wr_rm_depth_ahat(filename, port, mode, profile, bitrate, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(hl2ss._create_configuration_for_rm_depth_ahat(mode, profile, bitrate))
    w.put(_create_user(user))
    return w


def _create_wr_rm_depth_longthrow(filename, port, mode, png_filter, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(hl2ss._create_configuration_for_rm_depth_longthrow(mode, png_filter))
    w.put(_create_user(user))
    return w


def _create_wr_rm_imu(filename, port, mode, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(hl2ss._create_configuration_for_rm_imu(mode))
    w.put(_create_user(user))
    return w


def _create_wr_pv(filename, port, mode, width, height, framerate, profile, bitrate, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(hl2ss._create_configuration_for_pv(mode, width, height, framerate, profile, bitrate))
    w.put(_create_user(user))
    return w


def _create_wr_microphone(filename, port, profile, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(hl2ss._create_configuration_for_microphone(profile))
    w.put(_create_user(user))
    return w


def _create_wr_si(filename, port, user):
    w = _writer()
    w.open(filename)
    w.put(_create_header(port))
    w.put(_create_user(user))
    return w


#------------------------------------------------------------------------------
# Writer Wrappers
#------------------------------------------------------------------------------

class wr_rm_vlc(hl2ss._context_manager):
    def __init__(self, filename, port, mode, profile, bitrate, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.profile = profile
        self.bitrate = bitrate
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_vlc(self.filename, self.port, self.mode, self.profile, self.bitrate, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_rm_depth_ahat(hl2ss._context_manager):
    def __init__(self, filename, port, mode, profile, bitrate, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.profile = profile
        self.bitrate = bitrate
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_depth_ahat(self.filename, self.port, self.mode, self.profile, self.bitrate, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_rm_depth_longthrow(hl2ss._context_manager):
    def __init__(self, filename, port, mode, png_filter, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.png_filter = png_filter
        self.user = user

    def open(self):
        self._wr = _create_wr_rm_depth_longthrow(self.filename, self.port, self.mode, self.png_filter, self.user)

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
    def __init__(self, filename, port, mode, width, height, framerate, profile, bitrate, user):
        self.filename = filename
        self.port = port
        self.mode = mode
        self.width = width
        self.height = height
        self.framerate = framerate
        self.profile = profile
        self.bitrate = bitrate
        self.user = user

    def open(self):
        self._wr = _create_wr_pv(self.filename, self.port, self.mode, self.width, self.height, self.framerate, self.profile, self.bitrate, self.user)

    def write(self, packet):
        self._wr.write(packet)

    def close(self):
        self._wr.close()


class wr_microphone(hl2ss._context_manager):
    def __init__(self, filename, port, profile, user):
        self.filename = filename
        self.port = port
        self.profile = profile
        self.user = user
    
    def open(self):
        self._wr = _create_wr_microphone(self.filename, self.port, self.profile, self.user)

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


#------------------------------------------------------------------------------
# Writer From Receiver
#------------------------------------------------------------------------------

def create_wr_from_rx(filename, rx, user):
    if   (rx.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return wr_rm_vlc(filename, rx.port, rx.mode, rx.profile, rx.bitrate, user)
    elif (rx.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return wr_rm_vlc(filename, rx.port, rx.mode, rx.profile, rx.bitrate, user)
    elif (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return wr_rm_vlc(filename, rx.port, rx.mode, rx.profile, rx.bitrate, user)
    elif (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return wr_rm_vlc(filename, rx.port, rx.mode, rx.profile, rx.bitrate, user)
    elif (rx.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return wr_rm_depth_ahat(filename, rx.port, rx.mode, rx.profile, rx.bitrate, user)
    elif (rx.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return wr_rm_depth_longthrow(filename, rx.port, rx.mode, rx.png_filter, user)
    elif (rx.port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return wr_rm_imu(filename, rx.port, rx.mode, user)
    elif (rx.port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return wr_rm_imu(filename, rx.port, rx.mode, user)
    elif (rx.port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return wr_rm_imu(filename, rx.port, rx.mode, user)
    elif (rx.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return wr_pv(filename, rx.port, rx.mode, rx.width, rx.height, rx.framerate, rx.profile, rx.bitrate, user)
    elif (rx.port == hl2ss.StreamPort.MICROPHONE):
        return wr_microphone(filename, rx.port, rx.profile, user)
    elif (rx.port == hl2ss.StreamPort.SPATIAL_INPUT):
        return wr_si(filename, rx.port, user)


def create_wr_from_producer(filename, producer, port, user):
    return create_wr_from_rx(filename, producer._rx[port], user)


#------------------------------------------------------------------------------
# File Reader
#------------------------------------------------------------------------------

class _reader:
    def open(self, filename, chunk):
        self._file = open(filename, 'rb')
        self._chunk = chunk
        
    def get(self, format):
        return struct.unpack(format, self._file.read(struct.calcsize(format)))
    
    def get_magic(self):
        return self.get(f'<{len(_MAGIC)}s')[0]
    
    def get_port(self):
        return self.get('<H')[0]
    
    def get_mode(self):
        return self.get('<B')[0]

    def get_video_format(self):
        return self.get('<HHB')

    def get_video_encoding(self):
        return self.get('<BI')

    def get_audio_encoding(self):
        return self.get('<B')[0]

    def get_png_encoding(self):
        return self.get('<B')[0]
    
    def get_user(self):
        return self._file.read(self.get('<I')[0])
    
    def begin(self, mode):
        self._unpacker = hl2ss._unpacker()
        self._unpacker.reset(mode)
        self._eof = False
        
    def read(self):
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


def _probe(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    r.close()
    return magic, port


#------------------------------------------------------------------------------
# Header Unpack
#------------------------------------------------------------------------------

class _header:
    def __init__(self, magic, port, user):
        self.magic = magic
        self.port = port
        self.user = user


class _header_rm_vlc(_header):
    def __init__(self, magic, port, mode, profile, bitrate, user):
        super().__init__(magic, port, user)
        self.mode = mode
        self.profile = profile
        self.bitrate = bitrate


class _header_rm_depth_ahat(_header):
    def __init__(self, magic, port, mode, profile, bitrate, user):
        super().__init__(magic, port, user)
        self.mode = mode
        self.profile = profile
        self.bitrate = bitrate


class _header_rm_depth_longthrow(_header):
    def __init__(self, magic, port, mode, png_filter, user):
        super().__init__(magic, port, user)
        self.mode = mode
        self.png_filter = png_filter


class _header_rm_imu(_header):
    def __init__(self, magic, port, mode, user):
        super().__init__(magic, port, user)
        self.mode = mode


class _header_pv(_header):
    def __init__(self, magic, port, mode, width, height, framerate, profile, bitrate, user):
        super().__init__(magic, port, user)
        self.mode = mode
        self.width = width
        self.height = height
        self.framerate = framerate
        self.profile = profile
        self.bitrate = bitrate


class _header_microphone(_header):
    def __init__(self, magic, port, profile, user):
        super().__init__(magic, port, user)
        self.profile = profile


class _header_si(_header):
    def __init__(self, magic, port, user):
        super().__init__(magic, port, user)


#------------------------------------------------------------------------------
# Mode 0 and Mode 1 Data Load
#------------------------------------------------------------------------------

def _create_rd_rm_vlc(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    mode = r.get_mode()
    profile, bitrate = r.get_video_encoding()
    user = r.get_user()
    r.begin(mode)
    return r, _header_rm_vlc(magic, port, mode, profile, bitrate, user)


def _create_rd_rm_depth_ahat(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    mode = r.get_mode()
    profile, bitrate = r.get_video_encoding()
    user = r.get_user()    
    r.begin(mode)
    return r, _header_rm_depth_ahat(magic, port, mode, profile, bitrate, user)


def _create_rd_rm_depth_longthrow(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    mode = r.get_mode()
    png_filter = r.get_png_encoding()
    user = r.get_user()
    r.begin(mode)
    return r, _header_rm_depth_longthrow(magic, port, mode, png_filter, user)


def _create_rd_rm_imu(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    mode = r.get_mode()
    user = r.get_user()
    r.begin(mode)
    return r, _header_rm_imu(magic, port, mode, user)


def _create_rd_pv(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    mode = r.get_mode()
    width, height, framerate = r.get_video_format()
    profile, bitrate = r.get_video_encoding()
    user = r.get_user()
    r.begin(mode)
    return r, _header_pv(magic, port, mode, width, height, framerate, profile, bitrate, user)


def _create_rd_microphone(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    profile = r.get_audio_encoding()
    user = r.get_user()
    r.begin(hl2ss.StreamMode.MODE_0)    
    return r, _header_microphone(magic, port, profile, user)


def _create_rd_si(filename, chunk):
    r = _reader()
    r.open(filename, chunk)
    magic = r.get_magic()
    port = r.get_port()
    user = r.get_user()
    r.begin(hl2ss.StreamMode.MODE_0)
    return r, _header_si(magic, port, user)


def _create_rd(filename, chunk):
    magic, port = _probe(filename, chunk)
    if   (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return _create_rd_rm_vlc(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return _create_rd_rm_vlc(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return _create_rd_rm_vlc(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return _create_rd_rm_vlc(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return _create_rd_rm_depth_ahat(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return _create_rd_rm_depth_longthrow(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return _create_rd_rm_imu(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return _create_rd_rm_imu(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return _create_rd_rm_imu(filename, chunk)
    elif (port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return _create_rd_pv(filename, chunk)
    elif (port == hl2ss.StreamPort.MICROPHONE):
        return _create_rd_microphone(filename, chunk)
    elif (port == hl2ss.StreamPort.SPATIAL_INPUT):
        return _create_rd_si(filename, chunk)


#------------------------------------------------------------------------------
# Reader Wrapper
#------------------------------------------------------------------------------

class _rd(hl2ss._context_manager):
    def __init__(self, filename, chunk):
        self.filename = filename
        self.chunk = chunk

    def open(self):
        self._rd, self.header = _create_rd(self.filename, self.chunk)

    def read(self):
        return self._rd.read()

    def close(self):
        self._rd.close()


#------------------------------------------------------------------------------
# Decoded Readers
#------------------------------------------------------------------------------

class _rd_decoded_rm_vlc(_rd):
    def __init__(self, filename, chunk):
        super().__init__(filename, chunk)

    def open(self):
        super().open()
        self._codec = hl2ss.decode_rm_vlc(self.header.profile)
        self._codec.create()
        self.read()

    def read(self):
        data = super().read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class _rd_decoded_rm_depth_ahat(_rd):
    def __init__(self, filename, chunk):
        super().__init__(filename, chunk)

    def open(self):
        super().open()
        self._codec = hl2ss.decode_rm_depth_ahat(self.header.profile)
        self._codec.create()
        self.read()

    def read(self):
        data = super().read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        super().close()


class _rd_decoded_rm_depth_longthrow(_rd):
    def __init__(self, filename, chunk):
        super().__init__(filename, chunk)

    def open(self):
        super().open()

    def read(self):
        data = super().read()
        if (data is not None):
            data.payload = hl2ss.decode_rm_depth_longthrow(data.payload)        
        return data

    def close(self):
        super().close()


class _rd_decoded_pv(_rd):
    def __init__(self, filename, chunk, format):
        super().__init__(filename, chunk)
        self.format = format

    def open(self):
        super().open()
        self._codec = hl2ss.decode_pv(self.header.profile)
        self._codec.create()
        self.read()

    def read(self):
        data = super().read()
        if (data is not None):
            data.payload = hl2ss.unpack_pv(data.payload)
            data.payload.image = self._codec.decode(data.payload.image, self.format)
        return data

    def close(self):
        super().close()


class _rd_decoded_microphone(_rd):
    def __init__(self, filename, chunk):
        super().__init__(filename, chunk)
        
    def open(self):
        super().open()
        self._codec = hl2ss.decode_microphone(self.header.profile)
        self._codec.create()

    def read(self):
        data = super().read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload)        
        return data

    def close(self):
        super().close()


#------------------------------------------------------------------------------
# Create Reader
#------------------------------------------------------------------------------

def create_rd(decoded, filename, chunk, format):
    magic, port = _probe(filename, chunk)
    if   (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return _rd_decoded_rm_vlc(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return _rd_decoded_rm_vlc(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return _rd_decoded_rm_vlc(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return _rd_decoded_rm_vlc(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return _rd_decoded_rm_depth_ahat(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return _rd_decoded_rm_depth_longthrow(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return _rd_decoded_pv(filename, chunk, format) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.MICROPHONE):
        return _rd_decoded_microphone(filename, chunk) if (decoded) else _rd(filename, chunk)
    elif (port == hl2ss.StreamPort.SPATIAL_INPUT):
        return _rd(filename, chunk)


#------------------------------------------------------------------------------
# Sequencer
#------------------------------------------------------------------------------

class sequencer:
    def __init__(self, decoded, filename, chunk, format):
        self.decoded = decoded
        self.filename = filename
        self.chunk = chunk
        self.format = format

    def open(self):
        self._rd = create_rd(self.decoded, self.filename, self.chunk, self.format)
        self._rd.open()
        self._l = self._rd.read()
        self._r = self._rd.read()

    def read(self, timestamp):
        if ((self._l is None) or (self._r is None)):
            return None
        if (timestamp < self._l.timestamp):
            return None
        while (timestamp > self._r.timestamp):
            self._l = self._r
            self._r = self._rd.read()
            if (self._r is None):
                return None
        return self._l if ((timestamp - self._l.timestamp) < (self._r.timestamp - timestamp)) else self._r
    
    def close(self):
        self._rd.close()


#------------------------------------------------------------------------------
# Background Writers
#------------------------------------------------------------------------------

def get_sync_period(wr):
    if   (wr.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss_mp.get_sync_period_rm_vlc(wr.profile)
    elif (wr.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss_mp.get_sync_period_rm_vlc(wr.profile)
    elif (wr.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss_mp.get_sync_period_rm_vlc(wr.profile)
    elif (wr.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss_mp.get_sync_period_rm_vlc(wr.profile)
    elif (wr.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return hl2ss_mp.get_sync_period_rm_depth_ahat(wr.profile)
    elif (wr.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return hl2ss_mp.get_sync_period_rm_depth_longthrow()
    elif (wr.port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return hl2ss_mp.get_sync_period_rm_imu()
    elif (wr.port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return hl2ss_mp.get_sync_period_rm_imu()
    elif (wr.port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return hl2ss_mp.get_sync_period_rm_imu()
    elif (wr.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return hl2ss_mp.get_sync_period_pv(wr.profile, wr.framerate)
    elif (wr.port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss_mp.get_sync_period_microphone()
    elif (wr.port == hl2ss.StreamPort.SPATIAL_INPUT):
        return hl2ss_mp.get_sync_period_si()


class wr_process_rx(mp.Process):
    def __init__(self, filename, rx, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = create_wr_from_rx(filename, rx, user)
        self._rx = rx

    def stop(self):
        self._event_stop.set()

    def run(self):
        self._wr.open()
        self._rx.open()
        while (not self._event_stop.is_set()):
            self._wr.write(self._rx.get_next_packet())
        self._rx.close()
        self._wr.close()


class wr_process_producer(mp.Process):
    def __init__(self, filename, producer, port, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = create_wr_from_producer(filename, producer, port, user)
        self._sink = hl2ss_mp.consumer().create_sink(producer, port, mp.Manager(), ...)

    def stop(self):
        self._event_stop.set()
        self._sink.release()
       
    def run(self):
        self._frame_stamp = hl2ss_mp.get_sync_frame_stamp(self._sink.get_attach_response() + 1, get_sync_period(self._wr))
        self._stop_stamp = self._frame_stamp + 1
        self._stopping = False

        self._wr.open()

        while (self._frame_stamp < self._stop_stamp):
            self._sink.acquire()
            state, data = self._sink.get_buffered_frame(self._frame_stamp)

            if (state == 0):
                self._frame_stamp += 1
                self._wr.write(data)
            elif (state < 0):
                break

            previous = self._stopping
            self._stopping = self._event_stop.is_set()
            if (not self._stopping):
                self._stop_stamp = self._frame_stamp + 1
            elif ((not previous) and self._stopping):
                self._stop_stamp = self._sink.get_frame_stamp()

        self._wr.close()
        self._sink.detach()
        

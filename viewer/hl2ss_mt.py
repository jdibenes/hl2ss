
import numpy as np
import weakref
import hl2ss
import hl2ss_ulm_stream


class PV_DecodedFormat:
    BGR  = 0
    RGB  = 1
    BGRA = 2
    RGBA = 3
    GRAY = 4
    ANY  = 254


class TimePreference:
    PREFER_PAST    = -1
    PREFER_NEAREST =  0
    PREFER_FUTURE  =  1


class Status:
    DISCARDED = -1
    OK        =  0
    WAIT      =  1


#------------------------------------------------------------------------------
# Packet Unpacker
#------------------------------------------------------------------------------

class _packet:
    def __init__(self, data):
        self.frame_stamp = data['frame_stamp']
        self.status = data['status']
        self.timestamp = data['timestamp']
        self.payload = data['payload']
        self.pose = data['pose']
        self._handle = data['_handle']
        self._f = weakref.finalize(self, lambda h : hl2ss_ulm_stream.release_packet(h), self._handle)

    def _destroy(self):
        if (self._handle is None):
            return
        self._f.detach()
        hl2ss_ulm_stream.release_packet(self._handle)
        self._handle = None


#------------------------------------------------------------------------------
# Receiver
#------------------------------------------------------------------------------

def create_configuration(port):
    return hl2ss_ulm_stream.create_configuration(port)


class _source(hl2ss._context_manager):
    def __init__(self, host, port, buffer_size, configuration, decoded):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.configuration = dict(configuration)
        self.decoded = decoded
        self._handle = None

    def open(self):
        if (self._handle is not None):
            return
        self._handle = hl2ss_ulm_stream.open_stream(self.host, self.port, self.buffer_size, self.configuration, int(self.decoded))
        self._f = weakref.finalize(self, lambda h : hl2ss_ulm_stream.release_stream(h), self._handle)

    def _unpack_payload(self, payload):
        return payload

    def _unpack_pose(self, pose):
        return np.frombuffer(pose, dtype=np.float32).reshape((4, 4)) if (pose.nbytes >= 64) else None

    def _unpack(self, packet):
        if (packet.status != Status.OK):
            packet.payload = None
            packet.pose    = None
        else:
            packet.payload = self._unpack_payload(packet.payload)
            packet.pose    = self._unpack_pose(packet.pose)
        return packet

    def get_by_index(self, frame_stamp):
        return self._unpack(_packet(hl2ss_ulm_stream.get_by_index(self._handle, frame_stamp)))
    
    def get_by_timestamp(self, timestamp, time_preference=TimePreference.PREFER_NEAREST, tiebreak_right=False):
        return self._unpack(_packet(hl2ss_ulm_stream.get_by_timestamp(self._handle, timestamp, int(time_preference), int(tiebreak_right))))
    
    def close(self):
        if (self._handle is None):
            return
        self._f.detach()
        hl2ss_ulm_stream.release_stream(self._handle)
        self._handle = None


class _rx(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, False)


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class _rx_decoded_rm_vlc(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_rm_vlc(hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class _rx_decoded_rm_depth_ahat(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_rm_depth_ahat(hl2ss.DepthProfile.SAME, hl2ss.VideoProfile.RAW, 0)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class _rx_decoded_rm_depth_longthrow(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_rm_depth_longthrow(hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class _rx_decoded_rm_imu(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_rm_imu()
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class _rx_decoded_pv(_source):
    def __init__(self, host, port, buffer_size, configuration, decoded):
        super().__init__(host, port, buffer_size, configuration, decoded)

    def open(self):
        self._decoded_format = self.decoded
        self._codec = hl2ss.decode_pv(hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        frame = self._codec.decode(payload, 'any')
        if (self._decoded_format == PV_DecodedFormat.ANY):
            return frame
        w = int(frame.resolution[0])
        h = int(frame.resolution[1])
        c = frame.image.size // (w * h)
        frame.image = frame.image.reshape((h, w, c) if (c > 1) else (h, w))
        return frame

    def close(self):
        super().close()


class _rx_decoded_microphone(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._profile = self.configuration['profile']
        self._level = self.configuration['level']
        super().open()

    def _unpack_payload(self, payload):
        return np.frombuffer(payload, dtype=np.float32).reshape((2, -1)) if (self._profile != hl2ss.AudioProfile.RAW) else np.frombuffer(payload, dtype=np.int16).reshape((1, -1)) if (self._level != hl2ss.AACLevel.L5) else np.frombuffer(payload, dtype=np.float32).reshape((1, -1))

    def close(self):
        super().close()


class _rx_decoded_si(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_si()
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class _rx_decoded_eet(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_eet()
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class _rx_decoded_extended_audio(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._profile = self.configuration['profile']
        self._level = self.configuration['level']
        super().open()

    def _unpack_payload(self, payload):
        return np.frombuffer(payload, dtype=np.float32).reshape((2, -1)) if (self._profile != hl2ss.AudioProfile.RAW) else np.frombuffer(payload, dtype=np.int16).reshape((1, -1)) if ((self._level & 0x80) == 0) else np.frombuffer(payload, dtype=np.int8).reshape((1, -1))

    def close(self):
        super().close()


class _rx_decoded_extended_depth(_source):
    def __init__(self, host, port, buffer_size, configuration):
        super().__init__(host, port, buffer_size, configuration, True)

    def open(self):
        self._codec = hl2ss.decode_extended_depth(hl2ss.DepthProfile.SAME)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

def rx_rm_vlc(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port) 
    return _rx_decoded_rm_vlc(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_rm_depth_ahat(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_rm_depth_ahat(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_rm_depth_longthrow(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_rm_depth_longthrow(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_rm_imu(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_rm_imu(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_pv(host, port, buffer_size=512, configuration=None, decoded=PV_DecodedFormat.BGR):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_pv(host, port, buffer_size, configuration, decoded) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_microphone(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_microphone(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_si(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_si(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_eet(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_eet(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_extended_audio(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_extended_audio(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def rx_extended_depth(host, port, buffer_size=512, configuration=None, decoded=True):
    if (configuration is None):
        configuration = create_configuration(port)
    return _rx_decoded_extended_depth(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


#------------------------------------------------------------------------------
# Stream
#------------------------------------------------------------------------------

class stream(hl2ss._context_manager):
    def __init__(self, rx, buffer_size=None, source_kind=None, semaphore=None):
        self.rx = rx

    def open(self):
        self._tag = self.rx.port
        self.rx

        self._producer = self.rx
        self._sink = self.rx

        self._producer.open()
        self._safs = -1

    def get_receiver(self):
        return self.rx
    
    def get_reference_frame_stamp(self):
        return self._safs

    def get_nearest(self, timestamp, time_preference=TimePreference.PREFER_NEAREST, tiebreak_right=False, select_data=True):
        p = self._sink.get_by_timestamp(timestamp, time_preference, tiebreak_right)
        return (p.frame_stamp, p) if (p.status == Status.OK) else (-1, None)
   
    def get_frame_stamp(self):
        p = self._sink.get_by_index(-1)
        return p.frame_stamp if (p.status == Status.OK) else -1

    def get_most_recent_frame(self):
        p = self._sink.get_by_index(-1)
        return (p.frame_stamp, p) if (p.status == Status.OK) else (-1, None)

    def get_buffered_frame(self, frame_stamp, select_data=True):
        p = self._sink.get_by_index(frame_stamp)
        return (p.status, p.frame_stamp, p) if (p.status == Status.OK) else (p.status, -1, None)
    
    def get_nearest_frame_stamp(self, timestamp, time_preference=TimePreference.PREFER_NEAREST, tiebreak_right=False):
        p = self._sink.get_by_timestamp(timestamp, time_preference, tiebreak_right)
        return p.frame_stamp if (p.status == Status.OK) else -1

    def close(self):
        self._producer.close()


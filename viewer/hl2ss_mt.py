
import numpy as np
import weakref
import hl2ss
import hl2ss_mx
import hl2ss_ulm_stream


class _PV_DecodedFormat:
    BGR  = 0
    RGB  = 1
    BGRA = 2
    RGBA = 3
    GRAY = 4
    ANY  = 254


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

def _create_configuration(port):
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
        if (packet.status != hl2ss_mx.Status.OK):
            packet.payload = None
            packet.pose    = None
        else:
            packet.payload = self._unpack_payload(packet.payload)
            packet.pose    = self._unpack_pose(packet.pose)
        return packet

    def get_by_index(self, frame_stamp):
        return self._unpack(_packet(hl2ss_ulm_stream.get_by_index(self._handle, frame_stamp)))
    
    def get_by_timestamp(self, timestamp, time_preference, tiebreak_right):
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
        if (self._decoded_format != _PV_DecodedFormat.ANY):
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
        self.dtype, self.shape = hl2ss.microphone_parameters(self.configuration['profile'], self.configuration['level'])
        super().open()

    def _unpack_payload(self, payload):
        return np.frombuffer(payload, dtype=self.dtype).reshape(self.shape)

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
        self.dtype, self.shape = hl2ss.extended_audio_parameters(self.configuration['profile'], self.configuration['level'])
        super().open()

    def _unpack_payload(self, payload):
        return np.frombuffer(payload, dtype=self.dtype).reshape(self.shape)

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

def _translate_rm_vlc(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']   = rx.chunk
    configuration['mode']    = rx.mode
    configuration['divisor'] = rx.divisor
    configuration['profile'] = rx.profile
    configuration['level']   = rx.level
    configuration['bitrate'] = rx.bitrate
    configuration['options'] = rx.options
    
    return _rx_decoded_rm_vlc(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_rm_depth_ahat(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']      = rx.chunk
    configuration['mode']       = rx.mode
    configuration['divisor']    = rx.divisor
    configuration['profile_z']  = rx.profile_z
    configuration['profile_ab'] = rx.profile_ab
    configuration['level']      = rx.level
    configuration['bitrate']    = rx.bitrate
    configuration['options']    = rx.options

    return _rx_decoded_rm_depth_ahat(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_rm_depth_longthrow(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']      = rx.chunk
    configuration['mode']       = rx.mode
    configuration['divisor']    = rx.divisor
    configuration['png_filter'] = rx.png_filter

    return _rx_decoded_rm_depth_longthrow(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_rm_imu(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk'] = rx.chunk
    configuration['mode']  = rx.mode

    return _rx_decoded_rm_imu(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_pv(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']     = rx.chunk
    configuration['mode']      = rx.mode
    configuration['width']     = rx.width
    configuration['height']    = rx.height
    configuration['framerate'] = rx.framerate
    configuration['divisor']   = rx.divisor
    configuration['profile']   = rx.profile
    configuration['level']     = rx.level
    configuration['bitrate']   = rx.bitrate
    configuration['options']   = rx.options

    return _rx_decoded_pv(host, port, buffer_size, configuration, _tlb.pv_decoded_format[rx.format]) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_microphone(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']   = rx.chunk
    configuration['profile'] = rx.profile
    configuration['level']   = rx.level

    return _rx_decoded_microphone(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_si(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk'] = rx.chunk

    return _rx_decoded_si(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_eet(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk'] = rx.chunk
    configuration['fps']   = rx.fps

    return _rx_decoded_eet(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_extended_audio(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']           = rx.chunk
    configuration['mixer_mode']      = rx.mixer_mode
    configuration['loopback_gain']   = rx.loopback_gain
    configuration['microphone_gain'] = rx.microphone_gain
    configuration['profile']         = rx.profile
    configuration['level']           = rx.level

    return _rx_decoded_extended_audio(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


def _translate_extended_depth(host, port, rx, buffer_size, configuration, decoded):
    configuration['chunk']       = rx.chunk
    configuration['media_index'] = rx.options[hl2ss.H26xEncoderProperty.HL2SSAPI_VideoMediaIndex]
    configuration['stride_mask'] = rx.options[hl2ss.H26xEncoderProperty.HL2SSAPI_VideoStrideMask]
    configuration['mode']        = rx.mode
    configuration['divisor']     = rx.divisor
    configuration['profile_z']   = rx.profile_z

    return _rx_decoded_extended_depth(host, port, buffer_size, configuration) if (decoded) else _rx(host, port, buffer_size, configuration)


class _tlb:
    pv_decoded_format = {
        'bgr24' : _PV_DecodedFormat.BGR,
        'rgb24' : _PV_DecodedFormat.RGB,
        'bgra'  : _PV_DecodedFormat.BGRA,
        'rgba'  : _PV_DecodedFormat.RGBA,
        'gray8' : _PV_DecodedFormat.GRAY,
        'any'   : _PV_DecodedFormat.ANY,
    }

    registry = [
        (hl2ss.rx_rm_vlc,             hl2ss.rx_decoded_rm_vlc,             _translate_rm_vlc),
        (hl2ss.rx_rm_depth_ahat,      hl2ss.rx_decoded_rm_depth_ahat,      _translate_rm_depth_ahat),
        (hl2ss.rx_rm_depth_longthrow, hl2ss.rx_decoded_rm_depth_longthrow, _translate_rm_depth_longthrow),
        (hl2ss.rx_rm_imu,             hl2ss.rx_decoded_rm_imu,             _translate_rm_imu),
        (hl2ss.rx_pv,                 hl2ss.rx_decoded_pv,                 _translate_pv),
        (hl2ss.rx_microphone,         hl2ss.rx_decoded_microphone,         _translate_microphone),
        (hl2ss.rx_si,                 hl2ss.rx_decoded_si,                 _translate_si),
        (hl2ss.rx_eet,                hl2ss.rx_decoded_eet,                _translate_eet),
        (hl2ss.rx_extended_audio,     hl2ss.rx_decoded_extended_audio,     _translate_extended_audio),
        (hl2ss.rx_extended_depth,     hl2ss.rx_decoded_extended_depth,     _translate_extended_depth),
    ]


def _translate(rx, buffer_size):
    for rx_t, rx_decoded_t, _translate_t in _tlb.registry:
        if (isinstance(rx, rx_t)):
            return _translate_t(rx.host, rx.port, rx, buffer_size, _create_configuration(rx.port), isinstance(rx, rx_decoded_t))


#------------------------------------------------------------------------------
# Sink
#------------------------------------------------------------------------------

class _sink:
    def __init__(self, source):
        self._source = source
        self._safs   = source.get_by_index(-1).frame_stamp

    def acquire(self, block=True):
        raise Exception('not implemented')

    def release(self):
        raise Exception('not implemented')

    def get_attach_response(self):
        return self._safs
        
    def detach(self):
        pass

    def get_nearest(self, timestamp, time_preference=hl2ss_mx.TimePreference.PREFER_NEAREST, tiebreak_right=False, select_data=True):
        data = self._source.get_by_timestamp(timestamp, time_preference, tiebreak_right)
        return (data.frame_stamp, data if (select_data) else None) if (data.status == hl2ss_mx.Status.OK) else (-1, None)

    def get_frame_stamp(self):
        _, frame_stamp, _ = self.get_buffered_frame(-1, False)
        return frame_stamp

    def get_most_recent_frame(self):
        _, frame_stamp, data = self.get_buffered_frame(-1, True)
        return frame_stamp, data
    
    def get_buffered_frame(self, frame_stamp, select_data=True):
        data = self._source.get_by_index(frame_stamp)
        return (data.status, data.frame_stamp, data if (select_data) else None) if (data.status == hl2ss_mx.Status.OK) else (data.status, data.frame_stamp, None)
    
    def get_nearest_frame_stamp(self, timestamp, time_preference=hl2ss_mx.TimePreference.PREFER_NEAREST, tiebreak_right=False):
        frame_stamp, _ = self.get_nearest(timestamp, time_preference, tiebreak_right, False)
        return frame_stamp

    def get_source_status(self):
        return True
    
    def get_source_string(self):
        return None


#------------------------------------------------------------------------------
# Producer
#------------------------------------------------------------------------------

class producer:
    def __init__(self):
        self._rx = dict()
        self._producer = dict()

    def configure(self, port, receiver):
        self._rx[port] = receiver

    def initialize(self, port, buffer_size=512, source_kind=None, default_sink_semaphore=None):
        self._producer[port] = _translate(self._rx[port], buffer_size)

    def start(self, port):        
        self._producer[port].open()

    def stop(self, port):
        self._producer[port].close()

    def get_receiver(self, port):
        return self._rx[port]
    
    def _attach_sink(self, port):
        return _sink(self._producer[port])

    def _get_default_sink(self, port):
        return _sink(self._producer[port])


#------------------------------------------------------------------------------
# Consumer
#------------------------------------------------------------------------------

class consumer:
    def __init__(self):
        pass

    def create_sink(self, producer, port, manager, semaphore=None):
        return producer._attach_sink(port)
    
    def get_default_sink(self, producer, port):
        return producer._get_default_sink(port)


#------------------------------------------------------------------------------
# Stream
#------------------------------------------------------------------------------

class stream(hl2ss._context_manager):
    def __init__(self, rx, buffer_size=512, source_kind=None, semaphore=None):
        self.rx = rx
        self.buffer_size = buffer_size
        self.source_kind = source_kind
        self.semaphore = semaphore

    def open(self):
        self._tag = self.rx.port

        self._producer = producer()
        self._producer.configure(self._tag, self.rx)
        self._producer.initialize(self._tag, self.buffer_size, self.source_kind, self.semaphore)
        self._producer.start(self._tag)

        self._consumer = consumer()
        self._sink = self._consumer.get_default_sink(self._producer, self._tag)
        self._safs = self._sink.get_attach_response()

    def get_receiver(self):
        return self._producer.get_receiver(self._tag)
    
    def create_sink(self, manager, semaphore=None):
        return self._consumer.create_sink(self._producer, self._tag, manager, semaphore)

    def acquire(self, block=True):
        return self._sink.acquire(block)

    def release(self):
        self._sink.release()

    def get_reference_frame_stamp(self):
        return self._safs

    def get_nearest(self, timestamp, time_preference=hl2ss_mx.TimePreference.PREFER_NEAREST, tiebreak_right=False, select_data=True):
        return self._sink.get_nearest(timestamp, time_preference, tiebreak_right, select_data)
   
    def get_frame_stamp(self):
        return self._sink.get_frame_stamp()

    def get_most_recent_frame(self):
        return self._sink.get_most_recent_frame()

    def get_buffered_frame(self, frame_stamp, select_data=True):
        return self._sink.get_buffered_frame(frame_stamp, select_data)
    
    def get_nearest_frame_stamp(self, timestamp, time_preference=hl2ss_mx.TimePreference.PREFER_NEAREST, tiebreak_right=False):
        return self._sink.get_nearest_frame_stamp(timestamp, time_preference, tiebreak_right)

    def get_source_status(self):
        return self._sink.get_source_status()
    
    def get_source_string(self):
        return self._sink.get_source_string()

    def close(self):
        self._sink.detach()
        self._producer.stop(self._tag)


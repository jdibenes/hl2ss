
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
    ANY  = 0xFF


class TimePreference:
    PREFER_PAST    = -1
    PREFER_NEAREST =  0
    PREFER_FUTURE  =  1


class Status:
    DISCARDED = -1
    OK        = 0
    WAIT      = 1


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
# Receiver Wrappers
#------------------------------------------------------------------------------

def create_configuration(port):
    return hl2ss_ulm_stream.create_configuration(port)


class _rx(hl2ss._context_manager):
    def __init__(self, host, port, buffer_size, configuration):
        self._host = host
        self._port = port
        self._buffer_size = buffer_size
        self._configuration = dict(configuration)
        self._handle = None

    def open(self):
        if (self._handle is not None):
            return
        self._handle = hl2ss_ulm_stream.open_stream(self._host, self._port, self._buffer_size, self._configuration)
        self._f = weakref.finalize(self, lambda h : hl2ss_ulm_stream.release_stream(h), self._handle)

    def _unpack_payload(self, payload):
        return payload

    def _unpack_pose(self, pose):
        return np.frombuffer(pose, dtype=np.float32).reshape((4, 4)) if (pose.nbytes >= 64) else None

    def _unpack(self, packet):
        if (packet.status != 0):
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
    
    def _get_pv_dimensions(self):
        return hl2ss_ulm_stream.get_pv_dimensions(self._handle)

    def close(self):
        if (self._handle is None):
            return
        self._f.detach()
        hl2ss_ulm_stream.release_stream(self._handle)
        self._handle = None


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_rm_vlc(_rx):
    def open(self):
        self._codec = hl2ss.decode_rm_vlc(hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class rx_decoded_rm_depth_ahat(_rx):
    _preamble = b'\x00\x00\x00\x00\x00\x00\x00\x00'

    def open(self):
        self._codec = hl2ss.decode_rm_depth_ahat(hl2ss.DepthProfile.SAME, hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(rx_decoded_rm_depth_ahat._preamble + payload)
    
    def close(self):
        super().close()


class rx_decoded_rm_depth_longthrow(_rx):
    def open(self):
        self._codec = hl2ss.decode_rm_depth_longthrow(hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class rx_decode_rm_imu(_rx):
    def open(self):
        self._codec = hl2ss.decode_rm_imu()
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class rx_decoded_pv(_rx):
    def open(self):
        self._codec = hl2ss.decode_pv(hl2ss.VideoProfile.RAW)
        super().open()

    def _unpack_payload(self, payload):
        frame = self._codec.decode(payload, 'any')
        if (self._configuration['decoded_format'] != PV_DecodedFormat.ANY):
            w = frame.resolution[0]
            h = frame.resolution[1]
            c = frame.image.size // (w * h)
            frame.image = frame.image.reshape((h, w, c) if (c > 1) else (h, w))
        return frame

    def close(self):
        super().close()


class rx_decoded_microphone(_rx):
    def _unpack_payload(self, payload):
        return np.frombuffer(payload, dtype=np.float32).reshape((2, -1)) if (self._configuration['profile'] != hl2ss.AudioProfile.RAW) else np.frombuffer(payload, dtype=np.int16).reshape((1, -1)) if (self._configuration['level'] != hl2ss.AACLevel.L5) else np.frombuffer(payload, dtype=np.float32).reshape((1, -1))


class rx_si(_rx):
    def open(self):
        self._codec = hl2ss.decode_si()
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class rx_eet(_rx):
    def open(self):
        self._codec = hl2ss.decode_eet()
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()


class rx_decoded_extended_audio(_rx):
    def _unpack_payload(self, payload):
        return np.frombuffer(payload, dtype=np.float32).reshape((2, -1)) if (self._configuration['profile'] != hl2ss.AudioProfile.RAW) else np.frombuffer(payload, dtype=np.int16).reshape((1, -1)) if ((self._configuration['level'] & 0x80) == 0) else np.frombuffer(payload, dtype=np.int8).reshape((1, -1))


class rx_decoded_extended_depth(_rx):
    def open(self):
        self._codec = hl2ss.decode_extended_depth(hl2ss.DepthProfile.SAME)
        super().open()

    def _unpack_payload(self, payload):
        return self._codec.decode(payload)
    
    def close(self):
        super().close()



import numpy as np
import hl2ss
import hl2ss_ulm_stream


class PV_DecodedFormat:
    BGR  = 0
    RGB  = 1
    BGRA = 2
    RGBA = 3
    GRAY = 4


class TimePreference:
    PREFER_PAST = -1
    PREFER_NEAREST = 0
    PREFER_FUTURE = 1


def create_configuration(port):
    return hl2ss_ulm_stream.create_configuration(port)


class _packet:
    def __init__(self, data):
        self.frame_stamp = data['frame_stamp']
        self.status = data['status']
        self.timestamp = data['timestamp']
        self.payload = data['payload']
        self.pose = data['pose']
        self._handle = data['_handle']

    def destroy(self):
        if (self._handle is None):
            return
        hl2ss_ulm_stream.release_packet(self._handle)
        self._handle = None
        
    def __del__(self):
        self.destroy()


class _rx(hl2ss._context_manager):
    def __init__(self, host, port, buffer_size, configuration):
        self._host = host
        self._port = port
        self._buffer_size = buffer_size
        self._configuration = dict(configuration)
        self._handle = None
        self._pv_wxh = False

    def open(self):
        if (self._handle is not None):
            return
        self._handle = hl2ss_ulm_stream.open_stream(self._host, self._port, self._buffer_size, self._configuration)

    def _unpack_payload(self, payload):
        return payload

    def _unpack_pose(self, pose):
        return np.frombuffer(pose, dtype=np.float32).reshape((4, 4)) if (pose.nbytes >= 64) else None

    def _unpack(self, packet):
        if (packet.status != 0):
            packet.payload = None
            packet.pose = None
        else:
            packet.payload = self._unpack_payload(packet.payload) # should copy?
            packet.pose = self._unpack_pose(packet.pose)  # should copy?
        return packet

    def get_by_index(self, frame_stamp):
        return self._unpack(_packet(hl2ss_ulm_stream.get_by_index(self._handle, frame_stamp)))
    
    def get_by_timestamp(self, timestamp, time_preference, tiebreak_right):
        return self._unpack(_packet(hl2ss_ulm_stream.get_by_timestamp(self._handle, timestamp, int(time_preference), int(tiebreak_right))))
    
    def _get_pv_dimensions(self):
        return hl2ss_ulm_stream.get_pv_dimensions(self._handle)

    def close(self):
        if (self._handle is None):
            return
        hl2ss_ulm_stream.release_stream(self._handle)
        self._handle = None

    def __del__(self):
        self.close()


def unpack_rm_vlc(payload):
    image        = np.frombuffer(payload, dtype=np.uint8,  offset=0,                                   count=hl2ss.Parameters_RM_VLC.PIXELS).reshape(hl2ss.Parameters_RM_VLC.SHAPE)
    sensor_ticks = np.frombuffer(payload, dtype=np.uint64, offset=hl2ss.Parameters_RM_VLC.PIXELS +  0, count=1)
    exposure     = np.frombuffer(payload, dtype=np.uint64, offset=hl2ss.Parameters_RM_VLC.PIXELS +  8, count=1)
    gain         = np.frombuffer(payload, dtype=np.uint32, offset=hl2ss.Parameters_RM_VLC.PIXELS + 16, count=1)

    return hl2ss._RM_VLC_Frame(image, sensor_ticks, exposure, gain)


def unpack_rm_depth_ahat_block(payload):
    depth        = np.frombuffer(payload, dtype=np.uint16, offset=0,                                                              count=hl2ss.Parameters_RM_DEPTH_AHAT.PIXELS).reshape(hl2ss.Parameters_RM_DEPTH_AHAT.SHAPE)
    ab           = np.frombuffer(payload, dtype=np.uint16, offset=hl2ss.Parameters_RM_DEPTH_AHAT.PIXELS * hl2ss._SIZEOF.WORD,     count=hl2ss.Parameters_RM_DEPTH_AHAT.PIXELS).reshape(hl2ss.Parameters_RM_DEPTH_AHAT.SHAPE)
    sensor_ticks = np.frombuffer(payload, dtype=np.uint64, offset=hl2ss.Parameters_RM_DEPTH_AHAT.PIXELS * hl2ss._SIZEOF.WORD * 2, count=1)

    return hl2ss._RM_Depth_Frame(depth, ab, sensor_ticks)


def unpack_rm_depth_longthrow_block(payload):
    depth        = np.frombuffer(payload, dtype=np.uint16, offset=0,                                                                   count=hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS).reshape(hl2ss.Parameters_RM_DEPTH_LONGTHROW.SHAPE)
    ab           = np.frombuffer(payload, dtype=np.uint16, offset=hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS * hl2ss._SIZEOF.WORD,     count=hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS).reshape(hl2ss.Parameters_RM_DEPTH_LONGTHROW.SHAPE)
    sensor_ticks = np.frombuffer(payload, dtype=np.uint64, offset=hl2ss.Parameters_RM_DEPTH_LONGTHROW.PIXELS * hl2ss._SIZEOF.WORD * 2, count=1)
    
    return hl2ss._RM_Depth_Frame(depth, ab, sensor_ticks)


def unpack_pv(payload, width, height, bpp):
    shape = (height, width, bpp)
    b = width * height * bpp

    image                 = np.frombuffer(payload, dtype=np.uint8,   offset=0,      count=b).reshape(shape)
    focal_length          = np.frombuffer(payload, dtype=np.float32, offset=b +  0, count=2)
    principal_point       = np.frombuffer(payload, dtype=np.float32, offset=b +  8, count=2)
    exposure_time         = np.frombuffer(payload, dtype=np.uint64,  offset=b + 16, count=1)
    exposure_compensation = np.frombuffer(payload, dtype=np.uint64,  offset=b + 24, count=2)
    lens_position         = np.frombuffer(payload, dtype=np.uint32,  offset=b + 40, count=1)
    focus_state           = np.frombuffer(payload, dtype=np.uint32,  offset=b + 44, count=1)
    iso_speed             = np.frombuffer(payload, dtype=np.uint32,  offset=b + 48, count=1)
    white_balance         = np.frombuffer(payload, dtype=np.uint32,  offset=b + 52, count=1)
    iso_gains             = np.frombuffer(payload, dtype=np.float32, offset=b + 56, count=2)
    white_balance_gains   = np.frombuffer(payload, dtype=np.float32, offset=b + 64, count=3)

    return hl2ss._PV_Frame(image, focal_length, principal_point, exposure_time, exposure_compensation, lens_position, focus_state, iso_speed, white_balance, iso_gains, white_balance_gains)


def unpack_microphone(payload, profile, level):
    return np.frombuffer(payload, dtype=np.float32).reshape((2, -1)) if (profile != hl2ss.AudioProfile.RAW) else np.frombuffer(payload, dtype=np.int16).reshape((1, -1)) if (level != hl2ss.AACLevel.L5) else np.frombuffer(payload, dtype=np.float32).reshape((1, -1))


def unpack_extended_audio(payload, profile):
    return np.frombuffer(payload, dtype=np.float32).reshape((2, -1)) if (profile != hl2ss.AudioProfile.RAW) else np.frombuffer(payload, dtype=np.int16).reshape((1, -1))


class rx_decoded_rm_vlc(_rx):
    def _unpack_payload(self, payload):
        return unpack_rm_vlc(payload)


class rx_decoded_rm_depth_ahat(_rx):
    def _unpack_payload(self, payload):
        return unpack_rm_depth_ahat_block(payload)


class rx_decoded_rm_depth_longthrow(_rx):
    def _unpack_payload(self, payload):
        return unpack_rm_depth_longthrow_block(payload)


class rx_rm_imu(_rx):
    pass


class rx_decoded_pv(_rx):
    _format_bpp = {
        PV_DecodedFormat.BGR  : 3,
        PV_DecodedFormat.RGB  : 3,
        PV_DecodedFormat.BGRA : 4,
        PV_DecodedFormat.RGBA : 4,
        PV_DecodedFormat.GRAY : 1
    }

    def _unpack_payload(self, payload):
        if (not self._pv_wxh):
            self._width, self._height = self._get_pv_dimensions()
            self._bpp = rx_decoded_pv._format_bpp[self._configuration['decoded_format'] % 5]
            self._pv_wxh = True
        return unpack_pv(payload, self._width, self._height, self._bpp)


class rx_decoded_microphone(_rx):
    def _unpack_payload(self, payload):
        return unpack_microphone(payload, self._configuration['profile'], self._configuration['level'])


class rx_si(_rx):
    pass


class rx_eet(_rx):
    pass


class rx_decoded_extended_audio(_rx):
    def _unpack_payload(self, payload):
        return unpack_extended_audio(payload, self._configuration['profile'])


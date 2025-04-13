
import weakref
import collections
import requests
import struct
import hl2ss


class StreamPort:
    LIVE      = 'live'
    LIVE_HIGH = 'live_high'
    LIVE_MED  = 'live_med'
    LIVE_LOW  = 'live_low'


class StreamKind:
    VIDEO = 1
    AUDIO = 2


class ChunkSize:
    MRC = 4096


#------------------------------------------------------------------------------
# Network Client
#------------------------------------------------------------------------------

class _client:
    def open(self, host, port, user, password, chunk_size, configuration):
        self._response = requests.get(f'https://{host}/api/holographic/stream/{port}.mp4', params=configuration, auth=(user, password), verify=False, stream=True)
        if (self._response.status_code != 200):
            self._response.close()
            self._response.raise_for_status()
        self._f = weakref.finalize(self, lambda r : r.close(), self._response)
        self._iterator = self._response.iter_content(chunk_size)

    def recv(self):
        return next(self._iterator)

    def close(self):
        self._f.detach()
        self._response.close()


#------------------------------------------------------------------------------
# Packet Unpacker
#------------------------------------------------------------------------------

class _box:
    def __init__(self, size, type, data):
        self.size = size
        self.type = type
        self.data = data


class _unpacker:
    def reset(self):
        self._buffer = bytearray()
        self._state  = 0

    def extend(self, data):
        self._buffer.extend(data)

    def unpack(self):
        length = len(self._buffer)

        if ((self._state == 0) and (length >= 8)):
            self._box_l  = struct.unpack('>I', self._buffer[0:4])[0]
            self._box_t  = self._buffer[4:8].decode()
            self._state  = 1

        if ((self._state == 1) and (length >= self._box_l)):
            self._box_d  = self._buffer[8:self._box_l]
            self._buffer = self._buffer[self._box_l:]
            self._state  = 0
            return True
        
        return False
        
    def get(self):
        return _box(self._box_l, self._box_t, self._box_d)


def _flatten_box(box):
    subboxes = []
    offset = 0
    while (offset < len(box.data)):
        size = struct.unpack('>I', box.data[offset:(offset+4)])[0]
        subboxes.append(_box(size, box.data[(offset+4):(offset+8)].decode(), box.data[(offset+8):(offset+size)]))
        offset += size
    return subboxes


#------------------------------------------------------------------------------
# Packet Gatherer
#------------------------------------------------------------------------------

def _avcc_to_annex_b(sample):
    offset = 0
    while (offset < len(sample)):
        branch = offset + 4 + struct.unpack('>I', sample[offset:(offset+4)])[0]
        sample[offset:offset+4] = b'\x00\x00\x00\x01'
        offset = branch
    return sample


def _raw_aac_to_adts(sample):
    header = b'\xFF\xF1\x4C' + struct.pack('>I', 0x800001EC | ((len(sample) + 7) << 13))
    return header + sample


def _compute_timestamp(ct, et, tb):
    return ((ct + et) * hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS) // tb


class _gatherer:
    def __init__(self):
        self._client   = _client()
        self._unpacker = _unpacker()

    def open(self, host, port, user, password, chunk_size, configuration):
        self._state = 0
        self._unpacker.reset()
        self._client.open(host, port, user, password, chunk_size, configuration)
        self._video_id = None
        self._audio_id = None
        self._video_ct = 0
        self._audio_ct = 0
        self._video_tb = 30000
        self._audio_tb = 48000
        self._video_et = 0
        self._audio_et = 0
        self._video_init = None

    def get_next_packet(self, wait=True):
        packets = []
        while (True):
            self._unpacker.extend(self._client.recv())
            while (self._unpacker.unpack()):
                box = self._unpacker.get()
                if (self._state == 0):
                    if (box.type == 'moov'):
                        for moov_box in _flatten_box(box):
                            if (moov_box.type == 'trak'):
                                for trak_box in _flatten_box(moov_box):
                                    if (trak_box.type == 'tkhd'):
                                        id = struct.unpack('>I', trak_box.data[12:16])[0]
                                    elif (trak_box.type == 'mdia'):
                                        for mdia_box in _flatten_box(trak_box):
                                            if (mdia_box.type == 'mdhd'):
                                                ct = struct.unpack('>I', mdia_box.data[4:8])[0]
                                                tb = struct.unpack('>I', mdia_box.data[12:16])[0]
                                            elif (mdia_box.type == 'minf'):
                                                for minf_box in _flatten_box(mdia_box):
                                                    if (minf_box.type == 'stbl'):
                                                        for stbl_box in _flatten_box(minf_box):
                                                            if (stbl_box.type == 'stsd'):
                                                                stbl_data = stbl_box.data
                                                                stbl_type = stbl_data[12:16].decode()
                                                                if (stbl_type == 'avc1'):
                                                                    self._video_id = id
                                                                    # Video/Audio ahead/delayed about ~1 second
                                                                    # Also observed in device portal player
                                                                    # Force 1 second delay on video stream
                                                                    self._video_ct = (ct + 1) * tb
                                                                    self._video_tb = tb
                                                                    
                                                                    sps_size_base = 108
                                                                    sps_size_end  = sps_size_base + 2
                                                                    sps_size_data = stbl_data[sps_size_base:sps_size_end]
                                                                    sps_size = struct.unpack('>H', sps_size_data)[0]
                                                                    sps_base = sps_size_end
                                                                    sps_end  = sps_base + sps_size
                                                                    sps_data = stbl_data[sps_base:sps_end]
                                                                    pps_size_base = sps_end + 1
                                                                    pps_size_end  = pps_size_base + 2
                                                                    pps_size_data = stbl_data[pps_size_base:pps_size_end]
                                                                    pps_size = struct.unpack('>H', pps_size_data)[0]
                                                                    pps_base = pps_size_end
                                                                    pps_end  = pps_base + pps_size
                                                                    pps_data = stbl_data[pps_base:pps_end]

                                                                    self._video_init = b'\x00\x00' + sps_size_data + sps_data + b'\x00\x00' + pps_size_data + pps_data
                                                                elif (stbl_type == 'mp4a'):
                                                                    self._audio_id = id
                                                                    self._audio_ct = ct * tb
                                                                    self._audio_tb = tb
                        self._state = 1
                elif (self._state == 1):
                    if (box.type == 'moof'):
                        self._streams = []
                        for moof_box in _flatten_box(box):
                            if (moof_box.type == 'traf'):
                                for traf_box in _flatten_box(moof_box):
                                    if (traf_box.type == 'tfhd'):
                                        id = struct.unpack('>I', traf_box.data[4:8])[0]
                                    elif (traf_box.type == 'trun'):
                                        sample_count = struct.unpack('>I', traf_box.data[4:8])[0]
                                        offset = struct.unpack('>i', traf_box.data[8:12])[0]
                                        sample_spans = [struct.unpack('>I', traf_box.data[12+(16*i)  :12+(16*i)+ 4])[0] for i in range(0, sample_count)]
                                        sample_sizes = [struct.unpack('>I', traf_box.data[12+(16*i)+4:12+(16*i)+ 8])[0] for i in range(0, sample_count)]
                                        sample_flags = [struct.unpack('>I', traf_box.data[12+(16*i)+8:12+(16*i)+12])[0] for i in range(0, sample_count)]
                                        self._streams.append((id, offset, sample_count, sample_spans, sample_sizes, sample_flags))
                        self._state = 2
                elif (self._state == 2):
                    if (box.type == 'mdat'):
                        self._streams.append((-1, len(box.data), 0, [], [], []))
                        for i in range(0, len(self._streams) - 1):
                            stream_l = self._streams[i]
                            stream_h = self._streams[i+1]
                            id = stream_l[0]
                            data = box.data[stream_l[1]:stream_h[1]]
                            count = stream_l[2]
                            spans = stream_l[3]
                            sizes = stream_l[4]
                            flags = stream_l[5]
                            offset = 0
                            for j in range(0, count):
                                span = spans[j]
                                size = sizes[j]
                                keyf = (~flags[j] >> 14) & 0x04
                                sample = data[offset:(offset+size)]
                                if (id == self._video_id):
                                    t = _compute_timestamp(self._video_ct, self._video_et, self._video_tb)
                                    if (self._video_init is not None):
                                        sample = sample[:6] + self._video_init + sample[6:] # AUD + SPS + PPS + IDR
                                        self._video_init = None
                                    packets.append(hl2ss._packet(t, _avcc_to_annex_b(sample) + struct.pack('B', StreamKind.VIDEO | keyf), None))
                                    self._video_et += span
                                elif (id == self._audio_id):
                                    t = _compute_timestamp(self._audio_ct, self._audio_et, self._audio_tb)
                                    packets.append(hl2ss._packet(t, _raw_aac_to_adts(sample) + struct.pack('B', StreamKind.AUDIO | keyf), None))
                                    self._audio_et += span
                                offset += size
                        self._state = 1
            if ((len(packets) > 0) or (not wait)):
                return packets
    
    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Stream Configuration
#------------------------------------------------------------------------------

def _bool_to_str(v):
    return 'true' if (v) else 'false'


def create_configuration_for_mrc(pv, holo, mic, loopback, RenderFromCamera, vstab, vstabbuffer):
    return {
        'holo' : _bool_to_str(holo), 
        'pv' :  _bool_to_str(pv), 
        'mic' : _bool_to_str(mic), 
        'loopback' : _bool_to_str(loopback), 
        'RenderFromCamera' : _bool_to_str(RenderFromCamera), 
        'vstab' : _bool_to_str(vstab),
        'vstabbuffer' : str(vstabbuffer)
    }


#------------------------------------------------------------------------------
# Mode 0 Data Acquisition
#------------------------------------------------------------------------------

def _connect_client_mrc(host, port, user, password, chunk_size, configuration):
    c = _gatherer()
    c.open(host, port, user, password, chunk_size, configuration)
    return c


#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

class rx_mrc(hl2ss._context_manager):
    def __init__(self, host, port, user, password, chunk, configuration):
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.chunk = chunk
        self.configuration = configuration

    def open(self):
        self._buffer = collections.deque()
        self._client = _connect_client_mrc(self.host, self.port, self.user, self.password, self.chunk, self.configuration)

    def get_next_packet(self, wait=True):
        if (len(self._buffer) <= 0):
            packets = self._client.get_next_packet(wait)
            if (len(packets) <= 0):
                return None
            self._buffer.extend(packets)
        return self._buffer.popleft()

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Decoder
#------------------------------------------------------------------------------

class MetadataSize:
    MRC = 1


class _MRC_Frame:
    def __init__(self, kind, sample, key_frame):
        self.kind      = kind
        self.sample    = sample
        self.key_frame = key_frame


class decode_mrc:
    def __init__(self):
        self._video_codec = hl2ss.get_video_codec(hl2ss.VideoProfile.H264_MAIN)
        self._audio_codec = hl2ss.get_audio_codec(hl2ss.AudioProfile.AAC_24000)

    def decode(self, payload, format):
        data   = payload[:-1]
        header = payload[-1:]

        flags     = struct.unpack('B', header)[0]
        kind      = flags & 3
        key_frame = (flags & 0x04) != 0
        sample    = data if (format is None) else self._video_codec.decode(data).to_ndarray(format=format) if (kind == StreamKind.VIDEO) else self._audio_codec.decode(data).to_ndarray() if (kind == StreamKind.AUDIO) else None

        return _MRC_Frame(kind, sample, key_frame)


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_mrc(rx_mrc):
    def __init__(self, host, port, user, password, chunk, configuration, format):
        super().__init__(host, port, user, password, chunk, configuration)
        self.format = format
        
    def open(self):
        self._codec = decode_mrc()
        super().open()

    def get_next_packet(self, wait=True):
        data = super().get_next_packet(wait)
        if (data is not None):
            data.payload = self._codec.decode(data.payload, self.format)
        return data

    def close(self):
        super().close()


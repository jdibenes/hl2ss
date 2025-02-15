
import requests
import struct
import hl2ss


class StreamPort:
    LIVE      = 'live'
    LIVE_HIGH = 'live_high'
    LIVE_MED  = 'live_med'
    LIVE_LOW  = 'live_low'


#------------------------------------------------------------------------------
# Network Client
#------------------------------------------------------------------------------

class _client:
    def open(self, host, port, chunk_size, user, password, configuration):
        self._response = requests.get(f'https://{host}/api/holographic/stream/{port}.mp4', params=configuration, auth=(user, password), verify=False, stream=True)
        self._iterator = self._response.iter_content(chunk_size)

    def recv(self):
        return next(self._iterator)

    def close(self):
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

        while (True):
            if (self._state == 0):
                if (length >= 8):
                    self._box_l = struct.unpack('>I', self._buffer[0:4])[0]
                    self._box_t = self._buffer[4:8].decode()
                    self._state = 1
                    continue
            elif (self._state == 1):
                if (length >= self._box_l):
                    self._box_d  = self._buffer[8:self._box_l]
                    self._buffer = self._buffer[self._box_l:]
                    self._state  = 0
                    return True
            return False
        
    def get(self):
        return _box(self._box_l, self._box_t, self._box_d)


def flatten_box(box):
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

def avcc_to_annex_b(sample):
    offset = 0
    while (offset < len(sample)):
        branch = offset + 4 + struct.unpack('>I', sample[offset:(offset+4)])[0]
        sample[offset:offset+4] = b'\x00\x00\x00\x01'
        offset = branch


def raw_aac_to_adts(sample):
    header = b'\xFF\xF1\x4C' + struct.pack('>I', 0x800001EC | ((len(sample) + 7) << 13))
    return header + sample


class _gatherer:
    def open(self, host, port, chunk_size, user, password, configuration):
        self._client = _client()
        self._unpacker = _unpacker()
        self._state = 0
        self._unpacker.reset()
        self._client.open(host, port, chunk_size, user, password, configuration)
        self._video_id = None
        self._audio_id = None

    def get_next_packet(self):
        packets = []
        while (True):
            self._unpacker.extend(self._client.recv())
            while (self._unpacker.unpack()):
                box = self._unpacker.get()
                if (self._state == 0):
                    if (box.type == 'moov'):
                        for moov_box in flatten_box(box):
                            if (moov_box.type == 'trak'):
                                for trak_box in flatten_box(moov_box):
                                    if (trak_box.type == 'tkhd'):
                                        id = struct.unpack('>I', trak_box.data[12:16])[0]
                                    elif (trak_box.type == 'mdia'):
                                        for mdia_box in flatten_box(trak_box):
                                            if (mdia_box.type == 'minf'):
                                                for minf_box in flatten_box(mdia_box):
                                                    if (minf_box.type == 'stbl'):
                                                        for stbl_box in flatten_box(minf_box):
                                                            if (stbl_box.type == 'stsd'):
                                                                stbl_data = stbl_box.data
                                                                stbl_type = stbl_data[12:16].decode()
                                                                if (stbl_type == 'avc1'):
                                                                    self._video_id = id
                                                                    sps_data = stbl_data[106:134]
                                                                    pps_data = stbl_data[133:141]
                                                                    sps_data[0:2] = b'\x00\x00'
                                                                    pps_data[0:2] = b'\x00\x00'
                                                                    xps_data = sps_data + pps_data
                                                                    avcc_to_annex_b(xps_data)
                                                                    packets.append((id, xps_data))
                                                                elif (stbl_type == 'mp4a'):
                                                                    self._audio_id = id
                        self._state = 1
                elif (self._state == 1):
                    if (box.type == 'moof'):
                        self._streams = []
                        for moof_box in flatten_box(box):
                            if (moof_box.type == 'traf'):
                                offset = 0
                                sample_sizes = []
                                for traf_box in flatten_box(moof_box):
                                    if (traf_box.type == 'tfhd'):
                                        id = struct.unpack('>I', traf_box.data[4:8])[0]
                                    elif (traf_box.type == 'trun'):
                                        sample_count = struct.unpack('>I', traf_box.data[4:8])[0]
                                        offset = struct.unpack('>i', traf_box.data[8:12])[0]
                                        sample_sizes = [struct.unpack('>I', traf_box.data[12+(16*i)+4:12+(16*i)+8])[0] for i in range(0, sample_count)]
                                self._streams.append((id, offset, sample_sizes))
                        self._state = 2
                elif (self._state == 2):
                    if (box.type == 'mdat'):
                        self._streams.append((-1, len(box.data), []))
                        for i in range(0, len(self._streams) - 1):
                            stream_l = self._streams[i]
                            stream_h = self._streams[i+1]
                            id = stream_l[0]
                            data = box.data[stream_l[1]:stream_h[1]]
                            sizes = stream_l[2]
                            if (id == self._video_id):
                                avcc_to_annex_b(data)
                                kind = 1
                            elif (id == self._audio_id):
                                kind = 2
                            else:
                                continue
                            if (len(sizes) <= 0):
                                sizes.append(len(data))
                            offset = 0
                            for size in sizes:
                                sample = data[offset:(offset+size)]
                                if (len(sample) > 0):
                                    if (id == self._audio_id):
                                        sample = raw_aac_to_adts(sample)
                                    packets.append((kind, sample))
                                offset += size
                        self._state = 1
            if (len(packets) > 0):
                return packets
    
    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Stream Configuration
#------------------------------------------------------------------------------

def bool_to_str(v):
    return 'true' if (v) else 'false'


def create_configuration_for_mrc(pv=True, holo=False, mic=True, loopback=False, RenderFromCamera=True, vstab=False, vstabbuffer=15):
    return {
        'holo' : bool_to_str(holo), 
        'pv' :  bool_to_str(pv), 
        'mic' : bool_to_str(mic), 
        'loopback' : bool_to_str(loopback), 
        'RenderFromCamera' : bool_to_str(RenderFromCamera), 
        'vstab' : bool_to_str(vstab),
        'vstabbuffer' : str(vstabbuffer)
    }


#------------------------------------------------------------------------------
# Mode 0 Data Acquisition
#------------------------------------------------------------------------------

def _connect_client_mrc(host, port, chunk_size, user, password, configuration):
    c = _gatherer()
    c.open(host, port, chunk_size, user, password, configuration)
    return c


#------------------------------------------------------------------------------
# Receiver Wrappers
#------------------------------------------------------------------------------

class rx_mrc(hl2ss._context_manager):
    def __init__(self, host, port, chunk, user, password, configuration):
        self.host = host
        self.port = port
        self.chunk = chunk
        self.user = user
        self.password = password
        self.configuration = configuration

    def open(self):
        self._client = _connect_client_mrc(self.host, self.port, self.chunk, self.user, self.password, self.configuration)

    def get_next_packet(self):
        return self._client.get_next_packet()

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Decoded Receivers
#------------------------------------------------------------------------------

class rx_decoded_mrc(rx_mrc):
    def __init__(self, host, port, chunk, user, password, configuration, format):
        super().__init__(host, port, chunk, user, password, configuration)
        self.format = format
        self._video_codec = hl2ss.decode_pv(hl2ss.VideoProfile.H264_MAIN)
        self._audio_codec = hl2ss.decode_microphone(hl2ss.AudioProfile.AAC_12000, hl2ss.AACLevel.L2)

    def open(self):
        self._video_codec.create(0, 0)
        self._audio_codec.create()
        super().open()

    def get_next_packet(self):
        packets = super().get_next_packet()
        decoded = []
        for packet in packets:
            kind = packet[0]
            payload = packet[1]
            if (kind == 1):
                frame = self._video_codec.decode(payload, self.format)
            elif (kind == 2):
                frame = self._audio_codec.decode(payload)
            else:
                continue
            if (frame is not None):
                decoded.append((kind, frame))
        return decoded

    def close(self):
        super().close()


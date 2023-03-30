
import numpy as np
import struct
import fractions
import cv2
import av
import hl2ss


#------------------------------------------------------------------------------
# File I/O
#------------------------------------------------------------------------------

class writer:
    def open(self, filename, mode):
        self._data = open(filename, 'wb')
        self._data.write(struct.pack('<B', mode))

    def write(self, packet):
        self._data.write(hl2ss.pack_packet(packet))

    def close(self):
        self._data.close()


class reader:
    def open(self, filename, chunk):
        self._data = open(filename, 'rb')        
        self.mode = struct.unpack('<B', self._data.read(hl2ss._SIZEOF.BYTE))[0]
        self._unpacker = hl2ss._unpacker()
        self._eof = False
        self._unpacker.reset(self.mode)
        self.chunk = chunk
        
    def read(self):
        while (True):
            if (self._unpacker.unpack()):
                return self._unpacker.get()
            if (self._eof):
                return None
            chunk = self._data.read(self.chunk)
            self._eof = len(chunk) < self.chunk
            self._unpacker.extend(chunk)

    def close(self):
        self._data.close()


#------------------------------------------------------------------------------
# Decoded Reader
#------------------------------------------------------------------------------

class rd_decoded_rm_vlc:
    def open(self, filename, chunk, profile):
        self._client = reader()
        self._client.open(filename, chunk)
        self._codec = hl2ss.decode_rm_vlc(profile)
        self._codec.create()
        self.read()

    def read(self):
        data = self._client.read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        self._client.close()


class rd_decoded_rm_depth_ahat:
    def open(self, filename, chunk, profile):
        self._client = reader()
        self._client.open(filename, chunk)
        self._codec = hl2ss.decode_rm_depth_ahat(profile)
        self._codec.create()
        self.read()

    def read(self):
        data = self._client.read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        self._client.close()


def unpack_rm_depth(payload):
    composite = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    h, w, _ = composite.shape
    interleaved = composite.view(np.uint16).reshape((h, w, 2))
    depth, ab = np.dsplit(interleaved, 2)
    return hl2ss._RM_Depth_Frame(depth.reshape((h, w)), ab.reshape((h, w)))


class rd_decoded_rm_depth_longthrow:
    def open(self, filename, chunk):
        self._client = reader()
        self._client.open(filename, chunk)

    def read(self):
        data = self._client.read()
        if (data is not None):
            data.payload = unpack_rm_depth(data.payload)
        return data

    def close(self):
        self._client.close()


class rd_decoded_pv:
    def open(self, filename, chunk_size, profile, format):
        self._client = reader()
        self._client.open(filename, chunk_size)
        self._codec = hl2ss.decode_pv(profile)
        self._codec.create()
        self._format = format
        self.read()

    def read(self):
        data = self._client.read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload, self._format)
        return data

    def close(self):
        self._client.close()


class rd_decoded_microphone:
    def open(self, filename, chunk_size, profile):
        self._client = reader()
        self._client.open(filename, chunk_size)
        self._codec = hl2ss.decode_microphone(profile)
        self._codec.create()

    def read(self):
        data = self._client.read()
        if (data is not None):
            data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        self._client.close()


#------------------------------------------------------------------------------
# Sequencer
#------------------------------------------------------------------------------

class sequencer:
    def __init__(self, reader):
        self._reader = reader

    def begin(self):
        self._l = self._reader.read()
        self._r = self._reader.read()

    def next(self, timestamp):
        if ((self._l is None) or (self._r is None)):
            return None
        if (timestamp < self._l.timestamp):
            return None
        while (timestamp > self._r.timestamp):
            self._l = self._r
            self._r = self._reader.read()
            if (self._r is None):
                return None
        return self._l if ((timestamp - self._l.timestamp) < (self._r.timestamp - timestamp)) else self._r


#------------------------------------------------------------------------------
# Unpack to MP4
#------------------------------------------------------------------------------

class entry_bin2mp4:
    def __init__(self, filename, codec_name, framerate):
        self.filename   = filename
        self.codec_name = codec_name
        self.framerate  = framerate


def unpack_to_mp4(filename, chunk_size, entries, time_base):
    n = len(entries)

    readers = [reader() for _ in range(0, n)]
    container = av.open(filename, mode='w')
    streams = [container.add_stream(entries[i].codec_name, rate=entries[i].framerate) for i in range(0, n)]
    codecs = [av.CodecContext.create(entries[i].codec_name, "r") for i in range(0, n)]

    [readers[i].open(entries[i].filename, chunk_size) for i in range(0, n)]

    base = hl2ss._RANGEOF.U64_MAX

    for i in range(0, n):
        data = readers[i].read()
        if (data is not None):
            if (data.timestamp < base):
                base = data.timestamp

    [readers[i].close() for i in range(0, n)]
    [readers[i].open(entries[i].filename, chunk_size) for i in range(0, n)]

    for i in range(0, n):
        while (True):
            data = readers[i].read()
            if (data is None):
                break
            for packet in codecs[i].parse(data.payload):
                packet.stream = streams[i]
                packet.pts = data.timestamp - base
                packet.dts = packet.pts
                packet.time_base = time_base
                container.mux(packet)

    container.close()
    [readers[i].close() for i in range(0, n)]


def unpack_to_mp4_time_base():
    return fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)


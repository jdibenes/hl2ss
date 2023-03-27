
import multiprocessing as mp
import struct
import fractions
import av
import hl2ss


#------------------------------------------------------------------------------
# File I/O
#------------------------------------------------------------------------------

class _Header:
    def __init__(self, header):
        self.port    = header[0]
        self.mode    = header[1]
        self.profile = header[2]


class writer:
    def open(self, filename, port, mode, profile):
        self._data = open(filename, 'wb')
        self._data.write(struct.pack('<HBB', port, mode, profile if (profile is not None) else hl2ss._RANGEOF.U8_MAX))

    def write(self, packet):
        self._data.write(hl2ss.pack_packet(packet))

    def close(self):
        self._data.close()


class reader:
    def open(self, filename, chunk_size):
        self._data = open(filename, 'rb')        
        self._header = _Header(struct.unpack('<HBB', self._data.read(hl2ss._SIZEOF.WORD + 2 * hl2ss._SIZEOF.BYTE)))
        self._unpacker = hl2ss._unpacker(self._header.mode)
        self._chunk_size = chunk_size
        self._eof = False

    def get_header(self):
        return self._header
        
    def read(self):
        while (True):
            if (self._unpacker.unpack()):
                return self._unpacker.get()
            if (self._eof):
                return None

            chunk = self._data.read(self._chunk_size)
            self._eof = len(chunk) < self._chunk_size
            self._unpacker.extend(chunk)

    def close(self):
        self._data.close()


#------------------------------------------------------------------------------
# File Information
#------------------------------------------------------------------------------

def probe_file(filename):
    reader = hl2ss.reader()
    reader.open(filename, hl2ss.ChunkSize.SINGLE_TRANSFER)
    header = reader.get_header()
    reader.close()
    return header


#------------------------------------------------------------------------------
# Decoded Reader
#------------------------------------------------------------------------------

class rd_decoded_rm_vlc:
    def open(self, filename, chunk_size):
        self._client = hl2ss.reader()
        self._client.open(filename, chunk_size)
        header = self._client.get_header()
        self._codec = hl2ss.decode_rm_vlc(header.profile)
        self._codec.create()

    def read(self):
        data = self._client.read()
        if (data is None):
            return None
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        self._client.close()


class rd_decoded_rm_depth_ahat:
    def open(self, filename, chunk_size):
        self._client = hl2ss.reader()
        self._client.open(filename, chunk_size)
        header = self._client.get_header()
        self._codec = hl2ss.decode_rm_depth_ahat(header.profile)
        self._codec.create()

    def read(self):
        data = self._client.read()
        if (data is None):
            return None
        data.payload = self._codec.decode(data.payload)
        return data

    def close(self):
        self._client.close()


class rd_decoded_rm_depth_longthrow:
    def open(self, filename, chunk_size):
        self._client = hl2ss.reader()
        self._client.open(filename, chunk_size)

    def read(self):
        data = self._client.read()
        if (data is None):
            return None
        data.payload = hl2ss.decode_rm_depth_longthrow(data.payload)
        return data

    def close(self):
        self._client.close()


class rd_decoded_pv:
    def open(self, filename, chunk_size, format):
        self._client = hl2ss.reader()
        self._client.open(filename, chunk_size)
        header = self._client.get_header()
        self._codec = hl2ss.decode_pv(header.profile)
        self._codec.create()
        self._format = format

    def read(self):
        data = self._client.read()
        if (data is None):
            return None
        data.payload = self._codec.decode(data.payload, self._format)
        return data

    def close(self):
        self._client.close()


class rd_decoded_microphone:
    def open(self, filename, chunk_size):
        self._client = hl2ss.reader()
        self._client.open(filename, chunk_size)
        header = self._client.get_header()
        self._codec = hl2ss.decode_microphone(header.profile)
        self._codec.create()

    def read(self):
        data = self._client.read()
        if (data is None):
            return None
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
        self.filename = filename
        self.codec_name = codec_name
        self.framerate = framerate


def unpack_to_mp4(filename, chunk_size, entries, time_base):
    n = len(entries)

    readers = [hl2ss.reader() for _ in range(0, n)]
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


def entry_mp4_rm_vlc(filename):
    header = probe_file(filename)
    return entry_bin2mp4(filename, hl2ss.get_video_codec_name(header.profile), hl2ss.Parameters_RM_VLC.FPS)


def entry_mp4_rm_depth_ahat(filename):
    header = probe_file(filename)
    return entry_bin2mp4(filename, hl2ss.get_video_codec_name(header.profile), hl2ss.Parameters_RM_DEPTH_AHAT.FPS)


def entry_mp4_pv(filename, framerate):
    header = probe_file(filename)
    return entry_bin2mp4(filename, hl2ss.get_video_codec_name(header.profile), framerate)


def entry_mp4_microphone(filename):
    header = probe_file(filename)
    return entry_bin2mp4(filename, hl2ss.get_audio_codec_name(header.profile), hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)


def unpack_to_mp4_time_base():
    return fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)


#------------------------------------------------------------------------------
# Background Writers
#------------------------------------------------------------------------------

class BinaryWriter(mp.Process):
    def __init__(self, filename, port, mode, profile, event_stop, receiver):
        super().__init__()
        self._filename = filename
        self._port = port
        self._mode = mode
        self._profile = profile
        self._event_stop = event_stop
        self._receiver = receiver

    def stop(self):
        self._event_stop.set()

    def run(self):
        self._writer = hl2ss.writer()
        self._writer.open(self._filename, self._port, self._mode, self._profile)
        self._receiver.open()
        while not self._event_stop.is_set():
            self._writer.write(self._receiver.get_next_packet())
        self._receiver.close()
        self._writer.close()


class SynchronizedBinaryWriter(mp.Process):
    def __init__(self, filename, port, mode, profile, event_stop, sink, start_at):
        super().__init__()
        self._filename = filename
        self._port = port
        self._mode = mode
        self._profile = profile
        self._event_stop = event_stop
        self._sink = sink
        self._frame_stamp = start_at

    def stop(self):
        self._event_stop.set()
        self._sink.release()

    def check_stop(self):
        previous = self._stopping
        self._stopping = self._event_stop.is_set()
        return previous != self._stopping

    def run(self):
        self._stopping = False
        self._writer = hl2ss.writer()
        self._writer.open(self._filename, self._port, self._mode, self._profile)

        while (not self._stopping) or (self._frame_stamp < stop_stamp):
            self._sink.acquire()
            state, data = self._sink.get_buffered_frame(self._frame_stamp)

            if self.check_stop():
                stop_stamp = self._sink.get_frame_stamp()

            if state == 0:
                self._frame_stamp += 1
                self._writer.write(data)
            elif state < 0:
                break

        self._writer.close()


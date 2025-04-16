
import multiprocessing as mp
import fractions
import av
import hl2ss
import hl2ss_io
import hl2ss_mx
import hl2ss_mp


#------------------------------------------------------------------------------
# Background Writer
#------------------------------------------------------------------------------

class _wr_process(mp.Process):
    def __init__(self, filename, producer, port, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = hl2ss_io.create_wr_from_rx(filename, producer.get_receiver(port), user)
        self._sink = hl2ss_mp.consumer().create_sink(producer, port, mp.Manager(), ...)

    def stop(self):
        self._event_stop.set()
        self._sink.release()

    def run(self):
        self._sync_period = hl2ss_mx.get_sync_period(self._wr)
        self._frame_stamp = hl2ss_mx.get_sync_frame_stamp(self._sink.get_attach_response() + 1, self._sync_period)
        self._worker_name = hl2ss.get_port_name(self._wr.port)

        with self._wr as writer:
            while ((not self._event_stop.is_set()) and self._sink.get_source_status()):
                self._sink.acquire()
                state, _, data = self._sink.get_buffered_frame(self._frame_stamp)
                if (state == hl2ss_mx.Status.OK):
                    self._frame_stamp += 1
                    writer.write(data)
                elif (state == hl2ss_mx.Status.DISCARDED):
                    self._frame_stamp = hl2ss_mx.get_sync_frame_stamp(self._frame_stamp + 1, self._sync_period)
                    print(f'[hl2ss_ds._wr_process] {self._worker_name} writer out of sync')
        
        source_string = self._sink.get_source_string()
        self._sink.detach()
        if (source_string is None):
            return
        print(f'[hl2ss_ds._wr_process] {self._worker_name} source was lost:')
        print(source_string)


class wr(hl2ss._context_manager):
    def __init__(self, filename, producer, port, user):
        self._worker = _wr_process(filename, producer, port, user)

    def open(self):
        self._worker.start()

    def close(self):
        self._worker.stop()
        self._worker.join()


#------------------------------------------------------------------------------
# Unpacking
#------------------------------------------------------------------------------

class av_codec_kind:
    VIDEO = 1
    AUDIO = 2


def get_av_codec_name(rd):
    if (rd.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss.get_video_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss.get_video_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss.get_video_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss.get_video_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return hl2ss.get_video_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss.get_audio_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return hl2ss.get_audio_codec_name(rd.profile)
    if (rd.port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return hl2ss.get_video_codec_name(rd.profile)
    
    return None


def get_av_framerate(rd):
    if (rd.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (rd.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (rd.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (rd.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return hl2ss.Parameters_RM_VLC.FPS
    if (rd.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return rd.framerate
    if (rd.port == hl2ss.StreamPort.MICROPHONE):
        return hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
    if (rd.port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
    if (rd.port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return rd.framerate
    
    return None


def get_av_codec_kind(rd):
    if (rd.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return av_codec_kind.VIDEO
    if (rd.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return av_codec_kind.VIDEO
    if (rd.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return av_codec_kind.VIDEO
    if (rd.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return av_codec_kind.VIDEO
    if (rd.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return av_codec_kind.VIDEO
    if (rd.port == hl2ss.StreamPort.MICROPHONE):
        return av_codec_kind.AUDIO
    if (rd.port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return av_codec_kind.AUDIO
    if (rd.port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return av_codec_kind.VIDEO
    
    return None


def unpack_to_mp4(input_filenames, output_filename):
    time_base = fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)

    readers = [hl2ss_io.sequencer(hl2ss_io.create_rd(input_filename, hl2ss.ChunkSize.SINGLE_TRANSFER, None)) for input_filename in input_filenames]
    for reader in readers:
        reader.open()

    container = av.open(output_filename, mode='w')
    streams = [container.add_stream(get_av_codec_name(reader.rd), rate=get_av_framerate(reader.rd)) for reader in readers]
    codecs = [av.CodecContext.create(get_av_codec_name(reader.rd), "r") for reader in readers]

    for stream in streams:
        stream.time_base = time_base

    base = hl2ss._RANGEOF.U64_MAX

    for reader in readers:
        data = reader.get_left()
        if ((get_av_codec_kind(reader.rd) == av_codec_kind.VIDEO) and (data is not None) and (data.timestamp < base)):
            base = data.timestamp

    for reader, codec, stream in zip(readers, codecs, streams):
        metadata_size = hl2ss.get_metadata_size(reader.rd.port)
        while (True):
            data = reader.get_left()
            if (data is None):
                break
            reader.advance()

            if (metadata_size > 0):
                payload = data.payload[:-metadata_size]
            else:
                payload = data.payload

            local_timestamp = data.timestamp - base
            if (local_timestamp < 0):
                continue

            for p in codec.parse(payload):
                p.stream, p.pts, p.dts, p.time_base = stream, local_timestamp, local_timestamp, time_base
                container.mux(p)

    container.close()
    for reader in readers:
        reader.close()


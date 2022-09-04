#------------------------------------------------------------------------------
# Combine pv.bin and microphone.bin into mp4.
#------------------------------------------------------------------------------

from fractions import Fraction

import hl2ss
import av

# Settings --------------------------------------------------------------------

path_pv = './pv.bin'
path_microphone = './microphone.bin'

output_file = './av.mp4'

#------------------------------------------------------------------------------

video_profile = hl2ss.VideoProfile.H264_BASE
audio_profile = hl2ss.AudioProfile.AAC_24000
framerate = 30

video_codec_name = hl2ss.get_video_codec_name(video_profile)
audio_codec_name = hl2ss.get_audio_codec_name(audio_profile)

codec_video = av.CodecContext.create(video_codec_name, 'r')
codec_audio = av.CodecContext.create(audio_codec_name, 'r')

reader_video = hl2ss.raw_reader()
reader_audio = hl2ss.raw_reader()

reader_video.open(path_pv, hl2ss.ChunkSize.SINGLE_TRANSFER)
reader_audio.open(path_microphone, hl2ss.ChunkSize.SINGLE_TRANSFER)

time_base = Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)

container = av.open(output_file, 'w')

stream_video = container.add_stream(video_codec_name, rate=framerate)
stream_audio = container.add_stream(audio_codec_name, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)

video_start = None

while (True):
    data_video = reader_video.read()
    if (data_video is None):
        break
    if (video_start is None):
        video_start = data_video.timestamp
    for packet in codec_video.parse(data_video.payload):
        packet.stream = stream_video
        packet.pts = data_video.timestamp - video_start
        packet.dts = packet.pts
        packet.time_base = time_base
        container.mux(packet)

while (True):
    data_audio = reader_audio.read()
    if (data_audio is None):
        break
    if ((data_audio.timestamp - video_start) < 0):
        continue
    for packet in codec_audio.parse(data_audio.payload):
        packet.stream = stream_audio
        packet.pts = data_audio.timestamp - video_start
        packet.dts = packet.pts
        packet.time_base = time_base
        container.mux(packet)

reader_video.close()
reader_audio.close()

container.close()

#------------------------------------------------------------------------------
# Device Portal MRC recording example. Data is recorded to MP4 file.
# Press esc to stop recording.
#------------------------------------------------------------------------------

from pynput import keyboard

import fractions
import av
import hl2ss
import hl2ss_dp
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.1.7"

# Port
port = hl2ss_dp.StreamPort.LIVE

# Device Portal login
user = 'user'
password = 'pass'

# MRC Configuration
pv = True # Enable PV video
holo = False # Enable Holograms on PV video
mic = True # Enable Microphone
loopback = False # Include application audio
render_from_camera = True # Render Holograms from PV perspective
vstab = False # Enable video stabilization
vstabbuffer = 15 # Video stabilization buffer latency in frames [0, 30]

# Output filename
filename_out = './data/mrc_test.mp4'

#------------------------------------------------------------------------------

video_fps = 15 if (port == hl2ss_dp.StreamPort.LIVE_LOW) else 30

listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

time_base = fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)
format_video = hl2ss.get_video_codec_name(hl2ss.VideoProfile.H264_MAIN)
format_audio = hl2ss.get_audio_codec_name(hl2ss.AudioProfile.AAC_24000)

container = av.open(filename_out, mode='w')
stream_video = container.add_stream(format_video, rate=video_fps)
stream_audio = container.add_stream(format_audio, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)
codec_video = av.CodecContext.create(format_video, "r")
codec_audio = av.CodecContext.create(format_audio, "r")
stream_video.time_base = time_base
stream_audio.time_base = time_base

first_video_timestamp = None
first_audio_timestamp = None
audio_pending = []

configuration = hl2ss_dp.create_configuration_for_mrc(pv, holo, mic, loopback, render_from_camera, vstab, vstabbuffer)

decoder = hl2ss_dp.decode_mrc()
client = hl2ss_lnm.rx_dp_mrc(host, port, user, password, configuration=configuration, decoded_format=None)
client.open()

print('recording...')

while (not listener.pressed()):
    data = client.get_next_packet()
    data.payload = decoder.decode(data.payload, None)
    if (data.payload.kind == hl2ss_dp.StreamKind.VIDEO):
        if (first_video_timestamp is None):
            first_video_timestamp = data.timestamp
        t = data.timestamp - first_video_timestamp
        for p in codec_video.parse(data.payload.sample):
            p.stream, p.pts, p.dts, p.time_base = stream_video, t, t, time_base
            container.mux(p)
    elif (data.payload.kind == hl2ss_dp.StreamKind.AUDIO):
        audio_pending.append(data)
        if (first_video_timestamp is not None):
            for a in audio_pending:
                t = a.timestamp - first_video_timestamp
                if (t >= 0):
                    if (first_audio_timestamp is None):
                        first_audio_timestamp = a.timestamp
                    for p in codec_audio.parse(a.payload.sample):
                        p.stream, p.pts, p.dts, p.time_base = stream_audio, t, t, time_base
                        container.mux(p)
            audio_pending.clear()

client.close()
container.close()
listener.close()

print('done')

if ((first_video_timestamp is not None) and (first_audio_timestamp is not None)):
    print(f'Audio-Video offset is {(first_audio_timestamp - first_video_timestamp) / hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS} seconds')

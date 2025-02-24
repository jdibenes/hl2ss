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

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

format_video = 'h264'
format_audio = 'aac'

container = av.open(filename_out, mode='w')
stream_video = container.add_stream(format_video, rate=30)
stream_audio = container.add_stream(format_audio, rate=hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)
codec_video = av.CodecContext.create(format_video, "r")
codec_audio = av.CodecContext.create(format_audio, "r")

time_base = fractions.Fraction(1, hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS)

stream_video.time_base = time_base
stream_audio.time_base = time_base

configuration = hl2ss_dp.create_configuration_for_mrc(pv, holo, mic, loopback, render_from_camera, vstab, vstabbuffer)

client = hl2ss_lnm.rx_mrc(host, port, user, password, configuration=configuration, decoded_format=None)
client.open()

first_timestamp = None
audio_pending = []

while (enable):
    data = client.get_next_packet()
    data.payload = hl2ss_dp.unpack_mrc(data.payload)
    if (data.payload.kind == hl2ss_dp.StreamKind.VIDEO):
        if ((first_timestamp is None)):
            first_timestamp = data.timestamp
        t = data.timestamp - first_timestamp
        for p in codec_video.parse(data.payload.sample):
            p.stream = stream_video
            p.pts = t
            p.dts = t
            p.time_base = time_base
            container.mux(p)
    elif (data.payload.kind == hl2ss_dp.StreamKind.AUDIO):
        audio_pending.append(data)
        if (first_timestamp is None):
            continue
        for a in audio_pending:
            t = a.timestamp - first_timestamp
            if (t < 0):
                continue
            for p in codec_audio.parse(a.payload.sample):
                p.stream = stream_audio
                p.pts = t
                p.dts = t
                p.time_base = time_base
                container.mux(p)
        audio_pending.clear()

client.close()
container.close()
listener.join()

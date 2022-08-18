#------------------------------------------------------------------------------
# This script demonstrates how to acquire data from multiple streams and bring
# it into the main process.
#------------------------------------------------------------------------------

import multiprocessing as mp
import hl2ss
import hl2ss_mp
import av
import cv2

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Video encoding profile
profile = hl2ss.VideoProfile.H264_BASE

# Encoded stream average bits per second
# Must be > 0
bitrate = 2*1024*1024

# The internal ring buffer will hold buffer_size most recent packets
buffer_size = 1024

# Run for test_frames frames
test_frames = hl2ss.Parameters_RM_VLC.FPS * 10

#------------------------------------------------------------------------------

if __name__ == '__main__':
    manager = mp.Manager()

    receiver_lf = hl2ss.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT,  hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, profile, bitrate)
    receiver_ll = hl2ss.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT,   hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, profile, bitrate)
    receiver_rf = hl2ss.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, profile, bitrate)
    receiver_rr = hl2ss.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_0, profile, bitrate)

    receivers = [receiver_lf, receiver_ll, receiver_rf, receiver_rr]
    names = ['left front', 'left left', 'right front', 'right right']
    producers = []
    for receiver in receivers:
        producers.append(hl2ss_mp.producer(receiver, buffer_size))

    [p.start() for p in producers]

    sinks = []
    sink = None
    for producer in producers:
        sink = producer.create_sink(hl2ss_mp.get_sync_slot_rm_vlc(), manager, sink)
        sink.decoder = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
        sinks.append(sink)

    for i in range(0, len(sinks)):
        sinks[i].name = names[i]

    for sink in sinks:
        sink.get_attach_response()

    for sink in sinks:
        sink.wait_for_sync()

    frames = 0
    target_frames = test_frames * len(producers)
    control_sink = sinks[0]

    while (frames < target_frames):
        for sink in sinks:
            sink.target_frame_stamp = sink.get_frame_stamp() + 1

        for sink in sinks:
            for frame_stamp in range(sink.frame_stamp, sink.target_frame_stamp, 1):
                response, data = sink.get_buffered_frame(frame_stamp)
                if (response == 0):
                    for packet in sink.decoder.parse(data.payload):
                        for frame in sink.decoder.decode(packet):
                            cv2.imshow('Video ' + sink.name, frame.to_ndarray(format='bgr24'))
                elif (response < 0):
                    print('ERROR: consumer failed to keep up with producer')
                else:
                    print('BUG: requesting future frame')
            sink.frame_stamp = sink.target_frame_stamp

        cv2.waitKey(1)
        control_sink.wait_for_data()
        frames += 1

    for sink in sinks:
        sink.detach()

    [p.stop() for p in producers]

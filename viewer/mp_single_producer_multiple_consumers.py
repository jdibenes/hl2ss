#------------------------------------------------------------------------------
# This script demonstrates how to acquire data from a single stream in multiple
# processes.
#------------------------------------------------------------------------------

from time import sleep

import multiprocessing
import hl2ss
import hl2ss_mp
import av
import cv2

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Operating mode
# 0: video
# 1: video + camera pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Camera parameters
width = 1920
height = 1080
framerate = 30

# Video encoding profile
profile = hl2ss.VideoProfile.H264_MAIN

# Encoded stream average bits per second
# Must be > 0
bitrate = 5*1024*1024

# The internal ring buffer will hold buffer_size most recent packets
buffer_size = 1024

# Number of viewer processes
number_of_viewers = 4

# Run for at least delay seconds
delay = 20

#------------------------------------------------------------------------------

class Viewer(multiprocessing.Process):
    def __init__(self, index, profile, event_stop, sink):
        super().__init__()
        self._index = index
        self._profile = profile
        self._event_stop = event_stop
        self._sink = sink

    def stop(self):
        self._event_stop.set()
        self._sink.skip_wait()

    def run(self):
        codec = av.CodecContext.create(hl2ss.get_video_codec_name(self._profile), 'r')

        self._sink.get_attach_response()
        self._sink.wait_for_sync()

        while (not self._event_stop.is_set()):
            response, data = self._sink.get_buffered_frame(self._sink.frame_stamp)
            if (response == 0):
                self._sink.frame_stamp += 1
                for packet in codec.parse(data.payload):
                    for frame in codec.decode(packet):
                        cv2.imshow('Video {i}'.format(i=self._index), frame.to_ndarray(format='bgr24'))
            elif (response < 0):
                print('ERROR: consumer failed to keep up with producer')
            else:
                print('BUG: requesting future frame')

            cv2.waitKey(1)
            self._sink.wait_for_data()

        self._sink.detach()


if __name__ == "__main__":
    event_stop = multiprocessing.Event()

    receiver = hl2ss.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, mode, width, height, framerate, profile, bitrate)
    producer = hl2ss_mp.producer(receiver, buffer_size)
    
    viewers = []
    for i in range(0, number_of_viewers):
        sink = producer.create_sink(hl2ss_mp.get_sync_slot_pv(framerate), multiprocessing.Manager(), None)
        viewer = Viewer(i, profile, event_stop, sink)
        viewers.append(viewer)

    producer.start()

    [v.start() for v in viewers]

    sleep(delay)

    [v.stop() for v in viewers]
    [v.join() for v in viewers]

    producer.stop()


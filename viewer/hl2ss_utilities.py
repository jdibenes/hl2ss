
from pynput import keyboard

import multiprocessing as mp
import numpy as np
import time
import cv2
import av
import pyaudio
import hl2ss


#------------------------------------------------------------------------------
# Key Listener
#------------------------------------------------------------------------------

class key_listener:
    def __init__(self, key):
        self._key = key

    def _on_press(self, key):
        self._pressed = key == self._key
        return not self._pressed

    def open(self):
        self._pressed = False
        self._listener = keyboard.Listener(on_press=self._on_press)
        self._listener.start()

    def pressed(self):
        return self._pressed

    def close(self):
        self._pressed = True
        self._listener.join()


#------------------------------------------------------------------------------
# Microphone
#------------------------------------------------------------------------------

class _audio_process(mp.Process):
    IPC_BUFFER_READY = 0
    IPC_GET_TIMESTAMP = 1

    def __init__(self, subtype, planar, channels, sample_rate, buffer_frames):
        super().__init__()
        self._subtype       = subtype
        self._planar        = planar
        self._channels      = channels
        self._sample_rate   = sample_rate
        self._buffer_frames = buffer_frames
        self._pcm_queue     = mp.Queue()
        self._event_ready   = mp.Event()
        self._event_stop    = mp.Event()
        self._din           = mp.Queue()
        self._dout          = mp.Queue()
        self._semaphore     = mp.Semaphore(0)

    def stop(self):
        self._pcm_queue.put(None)
        if (not self._event_ready.is_set()):
            self._notify_queue()

    def put(self, timestamp, samples):
        self._pcm_queue.put((timestamp, samples))
        if (not self._event_ready.is_set()):
            self._buffer_frames -= 1
            if (self._buffer_frames > 0):
                return
            self._notify_queue()

    def pending(self):
        return self._pcm_queue.qsize()

    def get_timestamp(self):
        self._din.put((_audio_process.IPC_GET_TIMESTAMP,))
        self._semaphore.release()
        return self._dout.get()
    
    def _notify_queue(self):
        self._event_ready.set()
        self._din.put((_audio_process.IPC_BUFFER_READY,))
        self._semaphore.release()

    def _buffer_ready(self):
        self._stream = self._p.open(format=self._audio_format, channels=self._channels, rate=self._sample_rate, output=True, stream_callback=self._pcm_callback)
    
    def _get_timestamp(self):
        return self._presentation_clk
    
    def _process_control(self):
        try:
            message = self._din.get_nowait()
        except:
            return
        if (message[0] == _audio_process.IPC_BUFFER_READY):
            self._buffer_ready()
        if (message[0] == _audio_process.IPC_GET_TIMESTAMP):
            self._dout.put(self._get_timestamp())
        self._semaphore.acquire()

    def run(self):
        self._unit_time        = hl2ss.TimeBase.HUNDREDS_OF_NANOSECONDS // self._sample_rate
        self._pcm_audio_buffer = np.empty((1, 0), dtype=self._subtype)
        self._pcm_ts_buffer    = np.empty((1, 0), dtype=np.int64)
        self._presentation_clk = 0
        self._audio_format     = pyaudio.paFloat32 if (self._subtype == np.float32) else pyaudio.paInt16 if (self._subtype == np.int16) else None
        self._p                = pyaudio.PyAudio()
        
        while (not self._event_stop.is_set()):
            self._semaphore.acquire()
            self._semaphore.release()
            self._process_control()

        self._stream.close()

    def _pcm_callback(self, in_data, frame_count, time_info, status):
        samples = self._channels * frame_count

        while (self._pcm_ts_buffer.size < frame_count):
            pcm_data = self._pcm_queue.get()

            if (pcm_data is None):
                self._event_stop.set()
                self._semaphore.release()
                return (b'', pyaudio.paAbort)
            
            pcm_timestamp, pcm_payload = pcm_data

            pcm_samples    = hl2ss.microphone_planar_to_packed(pcm_payload, self._channels) if (self._planar) else pcm_payload
            pcm_group_size = pcm_samples.size // self._channels
            pcm_ts         = (pcm_timestamp + (np.arange(0, pcm_group_size, 1, dtype=np.int64) * self._unit_time)).reshape((1, -1))
            
            self._pcm_audio_buffer = np.hstack((self._pcm_audio_buffer, pcm_samples))
            self._pcm_ts_buffer    = np.hstack((self._pcm_ts_buffer,    pcm_ts))

        self._out_samples      = self._pcm_audio_buffer[:, :samples]
        self._pcm_audio_buffer = self._pcm_audio_buffer[:, samples:]
        self._presentation_clk = self._pcm_ts_buffer[0, 0]
        self._pcm_ts_buffer    = self._pcm_ts_buffer[:, frame_count:]

        return (self._out_samples.tobytes(), pyaudio.paContinue)


class audio_player:
    def __init__(self, subtype, planar, channels, sample_rate, buffer_frames=30):
        self.subtype       = subtype
        self.planar        = planar
        self.channels      = channels
        self.sample_rate   = sample_rate
        self.buffer_frames = buffer_frames

    def open(self):
        self._worker = _audio_process(self.subtype, self.planar, self.channels, self.sample_rate, self.buffer_frames)
        self._worker.start()

    def put(self, timestamp, samples):
        self._worker.put(timestamp, samples)

    def pending(self):
        return self._worker.pending()

    def get_timestamp(self):
        return self._worker.get_timestamp()

    def close(self):
        self._worker.stop()
        self._worker.join()


class microphone_resampler:
    def __init__(self, target_format=None, target_layout=None, target_rate=None):
        self._resampler = av.AudioResampler(format=target_format, layout=target_layout, rate=target_rate)

    def resample(self, data, profile):
        in_frame = av.AudioFrame.from_ndarray(data, format='s16' if (profile == hl2ss.AudioProfile.RAW) else 'fltp', layout='stereo')
        in_frame.rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
        out_frames = self._resampler.resample(in_frame)
        return [frame.to_ndarray() for frame in out_frames]


#------------------------------------------------------------------------------
# Draw
#------------------------------------------------------------------------------

def draw_points(image, points, radius, color, thickness):
    for x, y in points:
        if (x >= 0 and y >= 0 and x < image.shape[1] and y < image.shape[0]):
            cv2.circle(image, (x, y), radius, color, thickness)
    return image


#------------------------------------------------------------------------------
# Timing
#------------------------------------------------------------------------------

class continuity_analyzer:
    def reset(self, period):
        self._ub   = 1.5 * period
        self._lb   = 0.5 * period
        self._last = None

    def push(self, timestamp):
        if (self._last is None):
            status = (0, -1)
        else:
            delta = timestamp - self._last
            status = (1, delta) if (delta > self._ub) else (-1, delta) if (delta < self._lb) else (0, delta)
        self._last = timestamp
        return status


class framerate_counter:
    def reset(self):
        self._count = 0
        self._start = time.perf_counter()

    def increment(self):
        self._count += 1
        return self._count

    def delta(self):
        return time.perf_counter() - self._start

    def get(self):
        return self._count / self.delta()


class stream_report:
    def __init__(self, notify_period, stream_period):
        self._np = notify_period
        self._ca = continuity_analyzer()
        self._fc = framerate_counter()
        self._ca.reset(stream_period)
        self._fc.reset()

    def _report_continuity(self, timestamp):
        status, delta = self._ca.push(timestamp)
        if (status != 0):
            print(f'[hl2ss_utilities.stream_report] discontinuity type {status} detected with delta time {delta}')

    def _report_framerate(self):
        self._fc.increment()
        if (self._fc.delta() >= self._np):
            print(f'[hl2ss_utilities.stream_report] framerate {self._fc.get()}')
            self._fc.reset()

    def push(self, timestamp):
        self._report_continuity(timestamp)
        self._report_framerate()


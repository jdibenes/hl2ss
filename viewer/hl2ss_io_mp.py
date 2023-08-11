
import multiprocessing as mp
import hl2ss
import hl2ss_mp
import hl2ss_io


#------------------------------------------------------------------------------
# Background Writers
#------------------------------------------------------------------------------

def get_sync_period(wrapper):
    if (wrapper.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return wrapper.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (wrapper.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return wrapper.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (wrapper.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return wrapper.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (wrapper.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return wrapper.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (wrapper.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return wrapper.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (wrapper.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return 1
    if (wrapper.port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return 1
    if (wrapper.port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return 1
    if (wrapper.port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return 1
    if (wrapper.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return wrapper.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (wrapper.port == hl2ss.StreamPort.MICROPHONE):
        return 1
    if (wrapper.port == hl2ss.StreamPort.SPATIAL_INPUT):
        return 1
    if (wrapper.port == hl2ss.StreamPort.EXTENDED_EYE_TRACKER):
        return 1


class wr_process_rx(mp.Process):
    def __init__(self, filename, rx, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = hl2ss_io.create_wr_from_rx(filename, rx, user)
        self._rx = rx

    def stop(self):
        self._event_stop.set()

    def on_open(self):
        pass

    def on_receive(self, data):
        pass

    def on_close(self):
        pass

    def run(self):
        self.on_open()
        self._wr.open()
        self._rx.open()
        while (not self._event_stop.is_set()):
            data = self._rx.get_next_packet()
            self._wr.write(data)
            self.on_receive(data)
        self._rx.close()
        self._wr.close()
        self.on_close()


class wr_process_producer(mp.Process):
    def __init__(self, filename, producer, port, user):
        super().__init__()
        self._event_stop = mp.Event()
        self._wr = hl2ss_io.create_wr_from_producer(filename, producer, port, user)
        self._sink = hl2ss_mp.consumer().create_sink(producer, port, mp.Manager(), ...)
        self._sync_period = get_sync_period(self._wr)

    def stop(self):
        self._event_stop.set()
        self._sink.release()
       
    def on_open(self):
        pass

    def on_receive(self, data):
        pass

    def on_fail(self):
        pass

    def on_close(self):
        pass

    def run(self):
        self._frame_stamp = hl2ss_mp.get_sync_frame_stamp(self._sink.get_attach_response() + 1, self._sync_period)
        self._stopping = False

        self.on_open()
        self._wr.open()

        while ((not self._stopping) or (self._frame_stamp < self._stop_stamp)):
            self._sink.acquire()
            state, data = self._sink.get_buffered_frame(self._frame_stamp)

            if (state == 0):
                self._frame_stamp += 1
                self._wr.write(data)
                self.on_receive(data)
            elif (state < 0):
                self.on_fail()
                break

            if ((not self._stopping) and self._event_stop.is_set()):
                self._stopping = True
                self._stop_stamp = self._sink.get_frame_stamp()

        self._wr.close()
        self._sink.detach()
        self.on_close()
        

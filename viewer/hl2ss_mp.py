
import multiprocessing as mp
import hl2ss


#------------------------------------------------------------------------------
# Buffer
#------------------------------------------------------------------------------

class RingBuffer:
    """Implements a ring-buffer with the different processing after it becomes full.
    Idea: https://www.oreilly.com/library/view/python-cookbook/0596001673/ch05s19.html
    """

    def __init__(self, size_max = 64):
        self.max = size_max
        self.data = []

    class __Full:
        def append(self, x):
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max

        def get(self):
            return self.data[self.cur:]+self.data[:self.cur]

        def last(self):
            return self.get()[-1]

        def length(self):
            return self.max

    def append(self, x):
        self.data.append(x)
        if (len(self.data) == self.max):
            self.cur = 0
            self.__class__ = self.__Full

    def get(self):
        return self.data

    def last(self):
        if (len(self.data) == 0):
            return None
        return self.get()[-1]

    def length(self):
        return len(self.data)


def get_nearest_packet(data, timestamp):
    n = len(data)

    if (n <= 0):
        return None
    elif (n == 1):
        return 0

    l = 0
    r = n - 1

    while ((r - l) > 1):
        i = (r + l) // 2
        t = data[i].timestamp
        if (t < timestamp):
            l = i
        elif (t > timestamp):
            r = i
        else:
            return i

    return l if (abs(data[l].timestamp - timestamp) < abs(data[r].timestamp - timestamp)) else r


#------------------------------------------------------------------------------
# Source
#------------------------------------------------------------------------------

class net_source:
    def __init__(self, source_dout):
        self.source_dout = source_dout


class source(mp.Process):
    def __init__(self, receiver, event_stop, source_wires, interconnect_wires):
        super().__init__()
        self._source = receiver
        self._event_stop = event_stop
        self._source_dout = source_wires.source_dout
        self._interconnect_semaphore = interconnect_wires.interconnect_semaphore

    def stop(self):
        self._event_stop.set()

    def run(self):
        self._source.open()
        while (not self._event_stop.is_set()):
            self._source_dout.put(self._source.get_next_packet())
            self._interconnect_semaphore.release()
        self._source.close()

    def get_interface(self):
        return net_source(self._source_dout)


def create_interface_source():
    return net_source(mp.Queue())


def create_source(receiver, source_wires, interconnect_wires):
    return source(receiver, mp.Event(), source_wires, interconnect_wires)


#------------------------------------------------------------------------------
# Interconnect
#------------------------------------------------------------------------------

class net_interconnect:
    def __init__(self, interconnect_din, interconnect_dout, interconnect_semaphore):
        self.interconnect_din = interconnect_din
        self.interconnect_dout = interconnect_dout
        self.interconnect_semaphore = interconnect_semaphore


class interconnect(mp.Process):
    IPC_SEMAPHORE_VALUE = 0
    IPC_CONTROL_ATTACH = 0
    IPC_SINK_DETACH = -1
    IPC_SINK_GET_NEAREST = -2
    IPC_SINK_GET_FRAME_STAMP = -3
    IPC_SINK_GET_MOST_RECENT_FRAME = -4
    
    def __init__(self, buffer_size, event_stop, source_wires, interconnect_wires):
        super().__init__()
        self._buffer_size = buffer_size
        self._event_stop = event_stop
        self._source_dout = source_wires.source_dout
        self._interconnect_din = interconnect_wires.interconnect_din
        self._interconnect_dout = interconnect_wires.interconnect_dout
        self._interconnect_semaphore = interconnect_wires.interconnect_semaphore

    def stop(self):
        self._event_stop.set()
        self._interconnect_semaphore.release()

    def attach_sink(self, sink_wires):
        self._interconnect_din.put(interconnect.IPC_CONTROL_ATTACH)
        self._interconnect_din.put(sink_wires.sink_din)
        self._interconnect_din.put(sink_wires.sink_dout)
        self._interconnect_din.put(sink_wires.sink_semaphore)
        self._interconnect_semaphore.release()
        
    def _attach(self):
        self._key += 1
        sink_din = self._interconnect_din.get()
        sink_dout = self._interconnect_din.get()
        sink_semaphore = self._interconnect_din.get()
        self._sink[self._key] = (sink_din, sink_dout, sink_semaphore)
        sink_din.put(self._key)
        sink_din.put(self._frame_stamp)
        
    def _detach(self, sink_din, sink_dout):
        key = sink_dout.get()
        self._remove.append(key)

    def _get_nearest(self, sink_din, sink_dout):
        timestamp = sink_dout.get()
        buffer = self._buffer.get()
        index = get_nearest_packet(buffer, timestamp)
        response = (None, None) if (index is None) else (self._frame_stamp - self._buffer.length() + 1 + index, buffer[index])
        sink_din.put(response[0])
        sink_din.put(response[1])

    def _get_frame_stamp(self, sink_din, sink_dout):
        sink_din.put(self._frame_stamp)

    def _get_most_recent_frame(self, sink_din, sink_dout):
        sink_din.put(self._buffer.last())

    def _get_buffered_frame(self, sink_din, sink_dout, frame_stamp):
        n = self._buffer.length()
        index = n - 1 - self._frame_stamp + frame_stamp
        response = (-1, None) if (index < 0) else (1, None) if (index >= n) else (0, self._buffer.get()[index])
        sink_din.put(response[0])
        sink_din.put(response[1])

    def _process_source(self):
        try:
            data = self._source_dout.get_nowait()
        except:
            return
        self._frame_stamp += 1
        self._buffer.append(data)
        for _, _, ipc in self._sink.values():
            if (ipc is not None):
                ipc.release()
        self._interconnect_semaphore.acquire()

    def _process_control(self):
        try:
            message = self._interconnect_din.get_nowait()
        except:
            return
        if (message == interconnect.IPC_CONTROL_ATTACH):
            self._attach()
        self._interconnect_semaphore.acquire()

    def _process_sink_message(self, sink_din, sink_dout):
        try:
            message = sink_dout.get_nowait()
        except:
            return
        if (message == interconnect.IPC_SINK_DETACH):
            self._detach(sink_din, sink_dout)
        elif (message == interconnect.IPC_SINK_GET_NEAREST):
            self._get_nearest(sink_din, sink_dout)
        elif (message == interconnect.IPC_SINK_GET_FRAME_STAMP):
            self._get_frame_stamp(sink_din, sink_dout)         
        elif (message == interconnect.IPC_SINK_GET_MOST_RECENT_FRAME):
            self._get_most_recent_frame(sink_din, sink_dout)        
        else:
            self._get_buffered_frame(sink_din, sink_dout, message)
        self._interconnect_semaphore.acquire()

    def _process_sink(self):
        self._remove = []
        for sink_din, sink_dout, _ in self._sink.values():
            self._process_sink_message(sink_din, sink_dout)
        for key in self._remove:
            self._sink.pop(key)

    def run(self):
        self._buffer = RingBuffer(self._buffer_size)
        self._frame_stamp = -1
        self._sink = dict()
        self._key = 0

        while (not self._event_stop.is_set()):
            self._interconnect_semaphore.acquire()
            self._interconnect_semaphore.release()
            self._process_source()
            self._process_control()
            self._process_sink()

    def get_interface(self):
        return net_interconnect(self._interconnect_din, self._interconnect_dout, self._interconnect_semaphore)


def create_interface_interconnect():
    return net_interconnect(mp.Queue(), mp.Queue(), mp.Semaphore(interconnect.IPC_SEMAPHORE_VALUE))


def create_interconnect(buffer_size, source_wires, interconnect_wires):
    return interconnect(buffer_size, mp.Event(), source_wires, interconnect_wires)


#------------------------------------------------------------------------------
# Sink
#------------------------------------------------------------------------------

class net_sink:
    def __init__(self, sink_din, sink_dout, sink_semaphore):
        self.sink_din = sink_din
        self.sink_dout = sink_dout
        self.sink_semaphore = sink_semaphore


class sink:
    def __init__(self, sink_wires, interconnect_wires):
        self._sink_din = sink_wires.sink_din
        self._sink_dout = sink_wires.sink_dout
        self._sink_semaphore = sink_wires.sink_semaphore
        self._interconnect_semaphore = interconnect_wires.interconnect_semaphore

    def get_attach_response(self):
        self._key = self._sink_din.get()
        frame_stamp = self._sink_din.get()
        return frame_stamp
        
    def detach(self):
        self._sink_dout.put(interconnect.IPC_SINK_DETACH)
        self._sink_dout.put(self._key)
        self._interconnect_semaphore.release()

    def get_nearest(self, timestamp):
        self._sink_dout.put(interconnect.IPC_SINK_GET_NEAREST)
        self._sink_dout.put(timestamp)
        self._interconnect_semaphore.release()
        frame_stamp = self._sink_din.get()
        data = self._sink_din.get()
        return (frame_stamp, data)

    def get_frame_stamp(self):
        self._sink_dout.put(interconnect.IPC_SINK_GET_FRAME_STAMP)
        self._interconnect_semaphore.release()
        frame_stamp = self._sink_din.get()
        return frame_stamp

    def get_most_recent_frame(self):
        self._sink_dout.put(interconnect.IPC_SINK_GET_MOST_RECENT_FRAME)
        self._interconnect_semaphore.release()
        data = self._sink_din.get()
        return data

    def get_buffered_frame(self, frame_stamp):
        self._sink_dout.put(frame_stamp)
        self._interconnect_semaphore.release()
        state = self._sink_din.get()
        data = self._sink_din.get()
        return (state, data)

    def get_interface(self):
        return net_sink(self._sink_din, self._sink_dout, self._sink_semaphore)


def create_interface_sink(sink_din, sink_dout, sink_semaphore):
    return net_sink(sink_din, sink_dout, sink_semaphore)


def create_sink(sink_wires, interconnect_wires):
    return sink(sink_wires, interconnect_wires)


#------------------------------------------------------------------------------
# Stream Sync Period
#------------------------------------------------------------------------------

def get_sync_period_independent():
    return 1


def get_sync_period_rm_vlc():
    return hl2ss.Parameters_RM_VLC.FPS


def get_sync_period_pv(framerate):
    return framerate


def get_sync_frame_stamp(frame_stamp, sync_period):
    return frame_stamp + ((sync_period - (frame_stamp % sync_period)) % sync_period)


#------------------------------------------------------------------------------
# Producer
#------------------------------------------------------------------------------

class producer:
    def __init__(self, receiver, buffer_size):
        source_wires = create_interface_source()
        interconnect_wires = create_interface_interconnect()
        self._source = create_source(receiver, source_wires, interconnect_wires)
        self._interconnect = create_interconnect(buffer_size, source_wires, interconnect_wires)

    def start(self):
        self._interconnect.start()
        self._source.start()

    def create_sink(self, sink_din, sink_dout, sink_semaphore):
        sink_wires = create_interface_sink(sink_din, sink_dout, sink_semaphore)
        sink = create_sink(sink_wires, self._interconnect.get_interface())
        self._interconnect.attach_sink(sink_wires)
        return sink

    def stop(self):
        self._source.stop()
        self._source.join()
        self._interconnect.stop()
        self._interconnect.join()


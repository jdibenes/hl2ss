
import multiprocessing


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
        if len(self.data) == self.max:
            self.cur = 0
            self.__class__ = self.__Full

    def get(self):
        return self.data

    def last(self):
        if len(self.data) == 0:
            return None
        return self.get()[-1]

    def length(self):
        return len(self.data)


#------------------------------------------------------------------------------
# Source
#------------------------------------------------------------------------------

class net_source:
    def __init__(self, source_dout):
        self.source_dout = source_dout


class net_interconnect:
    def __init__(self, interconnect_din, interconnect_dout, interconnect_semaphore):
        self.interconnect_din = interconnect_din
        self.interconnect_dout = interconnect_dout
        self.interconnect_semaphore = interconnect_semaphore


class source(multiprocessing.Process):
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


class interconnect(multiprocessing.Process):
    IPC_CONTROL_ATTACH = 0
    IPC_SINK_GET_FRAME_STAMP = -1
    IPC_SINK_GET_MOST_RECENT_FRAME = -2
    IPC_SINK_DETACH = -3

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
        self._remove.append(sink_dout.get())

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
        for ipc in self._sink.values():
            ipc[2].release()
        self._interconnect_semaphore.acquire()
  
    def _process_control(self):
        try:
            message = self._interconnect_din.get_nowait()
        except:
            return
        if (message == 0):
            self._attach()
        self._interconnect_semaphore.acquire()

    def _process_sink_message(self, sink_din, sink_dout):
        try:
            message = sink_dout.get_nowait()
        except:
            return
        if (message == interconnect.IPC_SINK_DETACH):
            self._detach(sink_din, sink_dout)
        elif (message == interconnect.IPC_SINK_GET_FRAME_STAMP):
            self._get_frame_stamp(sink_din, sink_dout)         
        elif (message == interconnect.IPC_SINK_GET_MOST_RECENT_FRAME):
            self._get_most_recent_frame(sink_din, sink_dout)
        else:
            self._get_buffered_frame(sink_din, sink_dout, message)
        self._interconnect_semaphore.acquire()

    def _process_sink(self):
        self._remove = []
        for ipc in self._sink.values():
            self._process_sink_message(ipc[0], ipc[1])
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


def create_interface_source():
    return net_source(multiprocessing.Queue())


def create_source(receiver, source_wires, interconnect_wires):
    return source(receiver, multiprocessing.Event(), source_wires, interconnect_wires)


def create_interface_interconnect():
    return net_interconnect(multiprocessing.Queue(), multiprocessing.Queue(), multiprocessing.Semaphore(0))


def create_interconnect(buffer_size, source_wires, interconnect_wires):
    return interconnect(buffer_size, multiprocessing.Event(), source_wires, interconnect_wires)


#------------------------------------------------------------------------------
# Sink
#------------------------------------------------------------------------------

class net_sink:
    def __init__(self, sink_din, sink_dout, sink_semaphore):
        self.sink_din = sink_din
        self.sink_dout = sink_dout
        self.sink_semaphore = sink_semaphore


class sink:
    def __init__(self, synchronize_interval, sink_wires, interconnect_wires):
        self._synchronize_interval = synchronize_interval
        self._sink_din = sink_wires.sink_din
        self._sink_dout = sink_wires.sink_dout
        self._sink_semaphore = sink_wires.sink_semaphore
        self._interconnect_semaphore = interconnect_wires.interconnect_semaphore

    def get_attach_response(self):
        key = self._sink_din.get()
        frame_stamp = self._sink_din.get() + 1
        return (key, frame_stamp, frame_stamp + ((self._synchronize_interval - (frame_stamp % self._synchronize_interval)) % self._synchronize_interval))

    def wait_for_data(self):
        self._sink_semaphore.acquire()

    def skip_wait(self):
        self._sink_semaphore.release()

    def detach(self, key):
        self._sink_dout.put(interconnect.IPC_SINK_DETACH)
        self._sink_dout.put(key)
        self._interconnect_semaphore.release()

    def get_frame_stamp(self):
        self._sink_dout.put(interconnect.IPC_SINK_GET_FRAME_STAMP)
        self._interconnect_semaphore.release()
        return self._sink_din.get()

    def get_most_recent_frame(self):
        self._sink_dout.put(interconnect.IPC_SINK_GET_MOST_RECENT_FRAME)
        self._interconnect_semaphore.release()
        return self._sink_din.get()

    def get_buffered_frame(self, frame_stamp):
        self._sink_dout.put(frame_stamp)
        self._interconnect_semaphore.release()
        state = self._sink_din.get()
        data = self._sink_din.get()
        return (state, data)

    def get_interface(self):
        return net_sink(self._sink_din, self._sink_dout, self._sink_semaphore)


def create_interface_sink(manager, parallel_sink_wires):
    return net_sink(manager.Queue(), manager.Queue(), manager.Semaphore(0) if (parallel_sink_wires is None) else parallel_sink_wires.sink_semaphore)


def create_sink(synchronize_interval, sink_wires, interconnect_wires):
    return sink(synchronize_interval, sink_wires, interconnect_wires)


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

    def create_sink(self, synchronize_interval, manager, parallel_sink_wires):
        sink_wires = create_interface_sink(manager, parallel_sink_wires)
        sink = create_sink(synchronize_interval, sink_wires, self._interconnect.get_interface())
        self._interconnect.attach_sink(sink_wires)
        return sink

    def stop(self):
        self._source.stop()
        self._source.join()
        self._interconnect.stop()
        self._interconnect.join()


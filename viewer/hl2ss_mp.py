
import multiprocessing as mp
import threading as mt
import queue
import traceback
import hl2ss


class TimePreference:
    PREFER_PAST    = -1
    PREFER_NEAREST =  0
    PREFER_FUTURE  =  1


class Status:
    DISCARDED = -1
    OK        =  0
    WAIT      =  1


class SourceKind:
    MP = 0
    MT = 1


#------------------------------------------------------------------------------
# Buffer
#------------------------------------------------------------------------------

class RingBuffer:
    '''
    Implements a ring-buffer with the different processing after it becomes full.
    Idea: https://www.oreilly.com/library/view/python-cookbook/0596001673/ch05s19.html
    '''

    def __init__(self, size_max=64):
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


def _get_packet_interval(data, timestamp, l, r):
    while ((r - l) > 1):
        i = (r + l) // 2
        t = data[i].timestamp
        if (t < timestamp):
            l = i
        elif (t > timestamp):
            r = i
        else:
            return (i, i)
        
    return (l, r)


def get_nearest_packet(data, timestamp, time_preference=TimePreference.PREFER_NEAREST, tiebreak_right=False):
    n = len(data)

    if (n <= 0):
        return None

    si = _get_packet_interval(data, timestamp, 0, n - 1)

    if (si[0] == si[1]):
        return si[0]
    
    t0 = data[si[0]].timestamp
    t1 = data[si[1]].timestamp

    if (timestamp <= t0):
        return si[0]
    if (timestamp >= t1):
        return si[1]
    
    if (time_preference == TimePreference.PREFER_PAST):
        return si[0]
    if (time_preference == TimePreference.PREFER_FUTURE):
        return si[1]
    
    d0 = timestamp - t0
    d1 = t1 - timestamp

    if (d0 < d1):
        return si[0]
    if (d0 > d1):
        return si[1]
    
    return si[1 if (tiebreak_right) else 0] 


#------------------------------------------------------------------------------
# Source
#------------------------------------------------------------------------------

class _net_source:
    def __init__(self, dout):
        self.dout = dout


class _source:
    def __init__(self, receiver, event_stop, source_wires, interconnect_wires):
        self._source = receiver
        self._event_stop = event_stop
        self._source_wires = source_wires
        self._interconnect_wires = interconnect_wires

    def stop(self):
        self._event_stop.set()

    def run(self):
        try:
            with self._source as client:
                while (not self._event_stop.is_set()):
                    self._source_wires.dout.put(client.get_next_packet())
                    self._interconnect_wires.semaphore.release()
        except:
            self._source_wires.dout.put(hl2ss._packet(None, traceback.format_exc(), None))
            self._interconnect_wires.semaphore.release()
            self._event_stop.wait()
        self._source_wires.dout.put(None)


class _mp_source(mp.Process):
    def __init__(self, receiver, event_stop, source_wires, interconnect_wires):
        super().__init__()
        self._source = _source(receiver, event_stop, source_wires, interconnect_wires)

    def stop(self):
        self._source.stop()

    def run(self):
        self._source.run()


class _mt_source(mt.Thread):
    def __init__(self, receiver, event_stop, source_wires, interconnect_wires):
        super().__init__()
        self._source = _source(receiver, event_stop, source_wires, interconnect_wires)

    def stop(self):
        self._source.stop()

    def run(self):
        self._source.run()


def _create_interface_source(source_kind):
    return _net_source(mp.Queue() if (source_kind == SourceKind.MP) else queue.Queue() if (source_kind == SourceKind.MT) else None)


def _create_source(receiver, source_wires, interconnect_wires, source_kind):
    return _mp_source(receiver, mp.Event(), source_wires, interconnect_wires) if (source_kind == SourceKind.MP) else _mt_source(receiver, mt.Event(), source_wires, interconnect_wires) if (source_kind == SourceKind.MT) else None


#------------------------------------------------------------------------------
# Interconnect
#------------------------------------------------------------------------------

class _net_interconnect:
    def __init__(self, din, dout, semaphore):
        self.din = din
        self.dout = dout
        self.semaphore = semaphore


class _interconnect(mp.Process):
    IPC_SEMAPHORE_VALUE = 0
    IPC_CONTROL_STOP = 0
    IPC_CONTROL_ATTACH = 1
    IPC_SINK_DETACH = 0
    IPC_SINK_GET_NEAREST = 1
    IPC_SINK_GET_FRAME_STAMP = 2
    IPC_SINK_GET_MOST_RECENT_FRAME = 3
    IPC_SINK_GET_BUFFERED_FRAME = 4
    IPC_SINK_GET_SOURCE_STRING = 5
    
    def __init__(self, receiver, buffer_size, event_stop, source_kind, interconnect_wires, sink_wires):
        super().__init__()
        self._receiver = receiver
        self._buffer_size = buffer_size
        self._event_stop = event_stop
        self._source_kind = source_kind
        self._interconnect_wires = interconnect_wires
        self._sink_wires = sink_wires

    def stop(self):
        self._interconnect_wires.din.put((_interconnect.IPC_CONTROL_STOP,))
        self._interconnect_wires.semaphore.release()

    def attach_sink(self, sink_wires):
        self._interconnect_wires.din.put((_interconnect.IPC_CONTROL_ATTACH, sink_wires))
        self._interconnect_wires.semaphore.release()
        
    def _attach(self, sink_wires):
        self._key += 1
        self._sink[self._key] = sink_wires
        if (not self._source_status):
            sink_wires.event.set()
            if (sink_wires.semaphore is not None):
                sink_wires.semaphore.release()
        sink_wires.din.put((self._key, self._frame_stamp))
        
    def _detach(self, key):
        self._remove.append(key)
        return None

    def _get_nearest(self, timestamp, time_preference, tiebreak_right):
        buffer = self._buffer.get()
        index = get_nearest_packet(buffer, timestamp, time_preference, tiebreak_right)
        return (None, None) if (index is None) else (self._frame_stamp - self._buffer.length() + 1 + index, buffer[index])

    def _get_frame_stamp(self):
        return self._frame_stamp

    def _get_most_recent_frame(self):
        return self._frame_stamp, self._buffer.last()

    def _get_buffered_frame(self, frame_stamp):
        if (frame_stamp < 0):
            frame_stamp = self._frame_stamp + frame_stamp + 1
        n = self._buffer.length()
        index = n - 1 - self._frame_stamp + frame_stamp
        return (-1, frame_stamp, None) if (index < 0) else (1, frame_stamp, None) if (index >= n) else (0, frame_stamp, self._buffer.get()[index])

    def _get_source_status(self):
        return self._source_status
    
    def _get_source_string(self):
        return self._source_string

    def _process_source(self):
        try:
            data = self._source_wires.dout.get_nowait()
        except:
            return
        if (data.timestamp is None):
            self._source_status = False
            self._source_string = data.payload
            for sink_wires in self._sink.values():
                sink_wires.event.set()
        else:
            self._frame_stamp += 1
            self._buffer.append(data)
        for sink_wires in self._sink.values():
            if (sink_wires.semaphore is not None):
                sink_wires.semaphore.release()
        self._interconnect_wires.semaphore.acquire()

    def _process_control(self):
        try:
            message = self._interconnect_wires.din.get_nowait()
        except:
            return
        if   (message[0] == _interconnect.IPC_CONTROL_STOP):
            self._event_stop.set()
        elif (message[0] == _interconnect.IPC_CONTROL_ATTACH):
            self._attach(*message[1:])
        self._interconnect_wires.semaphore.acquire()

    def _process_sink_message(self, sink_wires):
        try:
            message = sink_wires.dout.get_nowait()
        except:
            return
        if   (message[0] == _interconnect.IPC_SINK_DETACH):
            sink_wires.din.put(self._detach(*message[1:]))
        elif (message[0] == _interconnect.IPC_SINK_GET_NEAREST):
            sink_wires.din.put(self._get_nearest(*message[1:]))
        elif (message[0] == _interconnect.IPC_SINK_GET_FRAME_STAMP):
            sink_wires.din.put(self._get_frame_stamp())
        elif (message[0] == _interconnect.IPC_SINK_GET_MOST_RECENT_FRAME):
            sink_wires.din.put(self._get_most_recent_frame())
        elif (message[0] == _interconnect.IPC_SINK_GET_BUFFERED_FRAME):
            sink_wires.din.put(self._get_buffered_frame(*message[1:]))
        elif (message[0] == _interconnect.IPC_SINK_GET_SOURCE_STRING):
            sink_wires.din.put(self._get_source_string())
        self._interconnect_wires.semaphore.acquire()

    def _process_sink(self):
        self._remove = []
        for sink_wires in self._sink.values():
            self._process_sink_message(sink_wires)
        for key in self._remove:
            self._sink.pop(key)

    def _process_flush(self):
        while (self._source_wires.dout.get() != None):
            pass

    def run(self):
        self._source_status = True
        self._source_string = None

        self._source_wires = _create_interface_source(self._source_kind)
        self._source = _create_source(self._receiver, self._source_wires, self._interconnect_wires, self._source_kind)
        self._source.start()

        self._buffer = RingBuffer(self._buffer_size)
        self._frame_stamp = -1
        
        self._sink = dict()
        self._key = 0
        self._attach(self._sink_wires)
        
        while (not self._event_stop.is_set()):       
            self._interconnect_wires.semaphore.acquire()
            self._interconnect_wires.semaphore.release()                
            self._process_source()
            self._process_control()
            self._process_sink()

        self._source.stop()
        self._process_flush()
        self._source.join()        


def _create_interface_interconnect():
    return _net_interconnect(mp.Queue(), mp.Queue(), mp.Semaphore(_interconnect.IPC_SEMAPHORE_VALUE))


def _create_interconnect(receiver, buffer_size, source_kind, interconnect_wires, sink_wires):
    return _interconnect(receiver, buffer_size, mp.Event(), source_kind, interconnect_wires, sink_wires)


#------------------------------------------------------------------------------
# Sink
#------------------------------------------------------------------------------

class _net_sink:
    def __init__(self, din, dout, semaphore, event):
        self.din = din
        self.dout = dout
        self.semaphore = semaphore
        self.event = event


class _sink:
    def __init__(self, sink_wires, interconnect_wires):
        self._sink_wires = sink_wires
        self._interconnect_wires = interconnect_wires

    def acquire(self):
        self._sink_wires.semaphore.acquire()

    def release(self):
        self._sink_wires.semaphore.release()

    def get_attach_response(self):
        self._key, frame_stamp = self._sink_wires.din.get()
        return frame_stamp
        
    def detach(self):
        self._sink_wires.dout.put((_interconnect.IPC_SINK_DETACH, self._key))
        self._interconnect_wires.semaphore.release()
        self._sink_wires.din.get()

    def get_nearest(self, timestamp, time_preference=TimePreference.PREFER_NEAREST, tiebreak_right=False):
        self._sink_wires.dout.put((_interconnect.IPC_SINK_GET_NEAREST, timestamp, time_preference, tiebreak_right))
        self._interconnect_wires.semaphore.release()
        frame_stamp, data = self._sink_wires.din.get()
        return frame_stamp, data

    def get_frame_stamp(self):
        self._sink_wires.dout.put((_interconnect.IPC_SINK_GET_FRAME_STAMP,))
        self._interconnect_wires.semaphore.release()
        frame_stamp = self._sink_wires.din.get()
        return frame_stamp

    def get_most_recent_frame(self):
        self._sink_wires.dout.put((_interconnect.IPC_SINK_GET_MOST_RECENT_FRAME,))
        self._interconnect_wires.semaphore.release()
        frame_stamp, data = self._sink_wires.din.get()
        return frame_stamp, data

    def get_buffered_frame(self, frame_stamp):
        self._sink_wires.dout.put((_interconnect.IPC_SINK_GET_BUFFERED_FRAME, frame_stamp))
        self._interconnect_wires.semaphore.release()
        state, frame_stamp, data = self._sink_wires.din.get() 
        return state, frame_stamp, data
    
    def get_source_status(self):
        return not self._sink_wires.event.is_set()
    
    def get_source_string(self):
        self._sink_wires.dout.put((_interconnect.IPC_SINK_GET_SOURCE_STRING,))
        self._interconnect_wires.semaphore.release()
        string = self._sink_wires.din.get()
        return string


def _create_interface_sink(sink_din, sink_dout, sink_semaphore, sink_event):
    return _net_sink(sink_din, sink_dout, sink_semaphore, sink_event)


def _create_interface_sink_default(semaphore):
    return _net_sink(mp.Queue(), mp.Queue(), mp.Semaphore(_interconnect.IPC_SEMAPHORE_VALUE) if (semaphore is ...) else None, mp.Event())


def _create_sink(sink_wires, interconnect_wires):
    return _sink(sink_wires, interconnect_wires)


#------------------------------------------------------------------------------
# Module
#------------------------------------------------------------------------------

class _module:
    def __init__(self, receiver, buffer_size, source_kind, default_sink_semaphore):
        self._interconnect_wires = _create_interface_interconnect()
        self._default_sink_wires = _create_interface_sink_default(default_sink_semaphore)
        self._interconnect = _create_interconnect(receiver, buffer_size, source_kind, self._interconnect_wires, self._default_sink_wires)
        self._default_sink = _create_sink(self._default_sink_wires, self._interconnect_wires)

    def start(self):
        self._interconnect.start()

    def stop(self):
        self._interconnect.stop()
        self._interconnect.join()

    def attach_sink(self, sink_din, sink_dout, sink_semaphore, sink_event):
        sink_wires = _create_interface_sink(sink_din, sink_dout, sink_semaphore, sink_event)
        sink = _create_sink(sink_wires, self._interconnect_wires)
        self._interconnect.attach_sink(sink_wires)
        return sink
    
    def get_default_sink(self):
        return self._default_sink


#------------------------------------------------------------------------------
# Producer
#------------------------------------------------------------------------------

class producer:
    def __init__(self):
        self._rx = dict()
        self._producer = dict()

    def configure(self, port, receiver):
        self._rx[port] = receiver

    def initialize(self, port, buffer_size=512, source_kind=SourceKind.MT, default_sink_semaphore=None):
        self._producer[port] = _module(self._rx[port], buffer_size, source_kind, default_sink_semaphore)

    def start(self, port):        
        self._producer[port].start()

    def stop(self, port):
        self._producer[port].stop()

    def get_receiver(self, port):
        return self._rx[port]
    
    def _attach_sink(self, port, sink_din, sink_dout, sink_semaphore, sink_event):
        return self._producer[port].attach_sink(sink_din, sink_dout, sink_semaphore, sink_event)
    
    def _get_default_sink(self, port):
        return self._producer[port].get_default_sink()


#------------------------------------------------------------------------------
# Consumer
#------------------------------------------------------------------------------

class consumer:
    def __init__(self):
        self._sink_din = dict()
        self._sink_dout = dict()
        self._sink_semaphore = dict()
        self._sink_event = dict()

    def create_sink(self, producer, port, manager, semaphore=None):
        sink_din = manager.Queue()
        sink_dout = manager.Queue()
        sink_semaphore = None if (semaphore is None) else manager.Semaphore(_interconnect.IPC_SEMAPHORE_VALUE) if (semaphore is ...) else self._sink_semaphore[semaphore]
        sink_event = manager.Event()
        
        self._sink_din[port] = sink_din
        self._sink_dout[port] = sink_dout
        self._sink_semaphore[port] = sink_semaphore
        self._sink_event[port] = sink_event

        return producer._attach_sink(port, sink_din, sink_dout, sink_semaphore, sink_event)
    
    def get_default_sink(self, producer, port):
        return producer._get_default_sink(port)


#------------------------------------------------------------------------------
# Stream
#------------------------------------------------------------------------------

class stream(hl2ss._context_manager):
    def __init__(self, rx, buffer_size=512, source_kind=SourceKind.MT, semaphore=None):
        self.rx = rx
        self.buffer_size = buffer_size
        self.source_kind = source_kind
        self.semaphore = semaphore

    def open(self):
        self._tag = self.rx.port

        self._producer = producer()
        self._producer.configure(self._tag, self.rx)
        self._producer.initialize(self._tag, self.buffer_size, self.source_kind, self.semaphore)
        self._producer.start(self._tag)

        self._consumer = consumer()
        self._sink = self._consumer.get_default_sink(self._producer, self._tag)
        self._safs = self._sink.get_attach_response()

    def get_receiver(self):
        return self._producer.get_receiver(self._tag)
    
    def create_sink(self, manager, semaphore=None):
        return self._consumer.create_sink(self._producer, self._tag, manager, semaphore)

    def acquire(self):
        self._sink.acquire()

    def release(self):
        self._sink.release()

    def get_reference_frame_stamp(self):
        return self._safs

    def get_nearest(self, timestamp, time_preference=TimePreference.PREFER_NEAREST, tiebreak_right=False):
        return self._sink.get_nearest(timestamp, time_preference, tiebreak_right)
   
    def get_frame_stamp(self):
        return self._sink.get_frame_stamp()

    def get_most_recent_frame(self):
        return self._sink.get_most_recent_frame()

    def get_buffered_frame(self, frame_stamp):
        return self._sink.get_buffered_frame(frame_stamp)
    
    def get_source_status(self):
        return self._sink.get_source_status()
    
    def get_source_string(self):
        return self._sink.get_source_string()

    def close(self):
        self._sink.detach()
        self._producer.stop(self._tag)


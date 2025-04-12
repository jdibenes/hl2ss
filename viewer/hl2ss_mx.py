
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
            self.cur = (self.cur + 1) % self.max

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
# Stream Sync Period
#------------------------------------------------------------------------------

def get_sync_frame_stamp(frame_stamp, sync_period):
    return frame_stamp + ((sync_period - (frame_stamp % sync_period)) % sync_period)


def get_sync_period(rx):
    if (rx.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_DEPTH_AHAT):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
        return 1
    if (rx.port == hl2ss.StreamPort.RM_IMU_ACCELEROMETER):
        return 1
    if (rx.port == hl2ss.StreamPort.RM_IMU_GYROSCOPE):
        return 1
    if (rx.port == hl2ss.StreamPort.RM_IMU_MAGNETOMETER):
        return 1
    if (rx.port == hl2ss.StreamPort.PERSONAL_VIDEO):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.MICROPHONE):
        return 1
    if (rx.port == hl2ss.StreamPort.SPATIAL_INPUT):
        return 1
    if (rx.port == hl2ss.StreamPort.EXTENDED_EYE_TRACKER):
        return 1
    if (rx.port == hl2ss.StreamPort.EXTENDED_AUDIO):
        return 1
    if (rx.port == hl2ss.StreamPort.EXTENDED_VIDEO):
        return rx.options[hl2ss.H26xEncoderProperty.CODECAPI_AVEncMPVGOPSize]
    if (rx.port == hl2ss.StreamPort.EXTENDED_DEPTH):
        return 1


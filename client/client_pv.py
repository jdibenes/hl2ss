
import multiprocessing
import hl2ss
import cv2
import av

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


# Commands accepted by the receiver-application bridge, based on BufferedCamera from milly-capture
class IPC:
    CID_FRAME_STAMP = 0
    CID_FRAMES_AVAILABLE = 1 # lparam=frame_stamp
    CID_GET_MOST_RECENT_FRAME = 2
    CID_GET_BUFFERED_FRAME = 3 # lparam=frame_stamp
    CID_PAUSE = 4
    CID_RESUME = 5
    CID_START = 6
    CID_STOP = 7


# Subprocess: Receive video from the front RGB camera
def rx_pv(host, chunk, mode, width, height, framerate, profile, bitrate, dout, event_quit, event_dout, lock):
    codec = av.CodecContext.create(hl2ss.get_video_codec_name(profile), 'r')
    
    try:
        client = hl2ss.connect_client_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, chunk, mode, width, height, framerate, profile, bitrate)
        try:
            while (not event_quit.is_set()):
                data = client.get_next_packet()
                packets = codec.parse(data.payload)
                for packet in packets:
                    for frame in codec.decode(packet):
                        data.payload = frame.to_ndarray(format='bgr24')
                        lock.acquire()
                        dout.put(data)
                        event_dout.set()
                        lock.release()
        except:
            pass
        finally:
            client.close()
    except:
        pass

    dout.cancel_join_thread()


# Subprocess: Receive video from the depth camera
def rx_rm_depth(host, chunk, mode, dout, event_quit, event_dout, lock):
    try:
        client = hl2ss.connect_client_rm_depth(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, chunk, mode)    
        try:
            while (not event_quit.is_set()):
                data = client.get_next_packet()
                images = hl2ss.unpack_rm_depth(data.payload)
                data.payload = images
                lock.acquire()
                dout.put(data)
                event_dout.set()
                lock.release()
        except:
            pass
        finally:
            client.close()
    except:
        pass

    dout.cancel_join_thread()


# Subprocess: Receive microphone audio
def rx_mc(host, chunk, profile, dout, event_quit, event_dout, lock):
    codec = av.CodecContext.create(hl2ss.get_audio_codec_name(profile), 'r')

    try:
        client = hl2ss.connect_client_mc(host, hl2ss.StreamPort.MICROPHONE, chunk, profile)
        try:
            while (not event_quit.is_set()):
                data = client.get_next_packet()
                packets = codec.parse(data.payload)
                for packet in packets:
                    for frame in codec.decode(packet):
                        data.payload = frame.to_ndarray()
                        lock.acquire()
                        dout.put(data)
                        event_dout.set()
                        lock.release()
        except:
            pass
        finally:
            client.close()
    except:
        pass

    dout.cancel_join_thread()


# Subprocess: Receive Spatial Input data (head+eye+hands)
def rx_si(host, chunk, dout, event_quit, event_dout, lock):   
    try:
        client = hl2ss.connect_client_si(host, hl2ss.StreamPort.SPATIAL_INPUT, chunk)
        try:
            while (not event_quit.is_set()):
                data = client.get_next_packet()
                lock.acquire()
                dout.put(data)
                event_dout.set()
                lock.release()
        except:
            pass
        finally:
            client.close()
    except:
        pass

    dout.cancel_join_thread()


# Combine command id and parameter into a message
def make_msg(cid, lparam):
    return ((cid & 0xFF) << 24) | (lparam & 0xFFFFFF)


# Subprocess: Receiver-Application bridge
def ipc_rx(target, args, elements, din, dout, event_ipc, lock):
    frame_stamp  = -1
    frame_buffer = RingBuffer(elements)

    rx_buffer  = multiprocessing.Queue()
    event_quit = multiprocessing.Event()
    
    args = list(args)
    args.append(rx_buffer)
    args.append(event_quit)
    args.append(event_ipc)
    args.append(lock)
    args = tuple(args)

    enabled    = True
    paused     = False
    rx_process = multiprocessing.Process(target=target, args=args)

    rx_process.start()

    while (enabled):
        event_ipc.wait()
        lock.acquire()

        try:
            data = rx_buffer.get_nowait()
        except:
            data = None

        try:
            msg = din.get_nowait()
        except:
            msg = None

        event_ipc.clear()
        lock.release()

        if (data is not None and not paused):
            frame_buffer.append(data)
            frame_stamp += 1

        if (msg is not None):
            cid    = (msg >> 24) & 0x000000FF
            lparam =  msg        & 0x00FFFFFF

            if (cid == IPC.CID_FRAME_STAMP):
                dout.put(frame_stamp)
            elif (cid == IPC.CID_FRAMES_AVAILABLE):
                dout.put(frame_stamp >= lparam)
            elif (cid == IPC.CID_GET_MOST_RECENT_FRAME):
                dout.put(frame_buffer.last())
            elif (cid == IPC.CID_GET_BUFFERED_FRAME):
                index = lparam - frame_stamp
                if (abs(index) >= elements):
                    # log: not in memory
                    f = None
                else:
                    buffer = frame_buffer.get()
                    if (len(buffer) <= 0):
                        # log: no frames
                        f = None
                    else:
                        f = buffer[index-1]
                dout.put(f)
            elif (cid == IPC.CID_PAUSE):
                paused = True
            elif (cid == IPC.CID_RESUME):
                paused = False
            elif (cid == IPC.CID_STOP):
                enabled = False

    event_quit.set()
    rx_process.join()


# multiprocess buffered camera, all data is passed using message queues
class BufferedCamera:
    def __init__(self, target, args, ring_buffer_size):
        self._din = multiprocessing.Queue()
        self._dout = multiprocessing.Queue()
        self._event = multiprocessing.Event()
        self._lock = multiprocessing.Lock()
        self._process = multiprocessing.Process(target=ipc_rx, args=(target, args, ring_buffer_size, self._din, self._dout, self._event, self._lock))
        
    def frame_stamp(self):
        self._lock.acquire()
        self._din.put(make_msg(IPC.CID_FRAME_STAMP, 0))
        self._event.set()
        self._lock.release()
        return self._dout.get()

    def frames_available(self, frame_stamp):
        self._lock.acquire()
        self._din.put(make_msg(IPC.CID_FRAMES_AVAILABLE, frame_stamp))
        self._event.set()
        self._lock.release()
        return self._dout.get()

    def get_most_recent_frame(self):
        self._lock.acquire()
        self._din.put(make_msg(IPC.CID_GET_MOST_RECENT_FRAME, 0))
        self._event.set()
        self._lock.release()
        return self._dout.get()

    def get_buffered_frame(self, frame_stamp):
        self._lock.acquire()
        self._din.put(make_msg(IPC.CID_GET_BUFFERED_FRAME, frame_stamp))
        self._event.set()
        self._lock.release()
        return self._dout.get()

    def start_camera(self):
        self._process.start()

    def stop_camera(self):
        self._lock.acquire()
        self._din.put(make_msg(IPC.CID_STOP, 0))
        self._event.set()
        self._lock.release()
        self._process.join()

# Main process (application): gathers frames from RGB camera, depth camera, microphone and spatial input
def viewer():
    # Configuration
    host = '192.168.1.15'
    # RGB camera
    chunk_pv = 4096
    width_pv = 1920
    height_pv = 1080
    framerate_pv = 30
    profile_pv = hl2ss.VideoProfile.H265_MAIN
    bitrate_pv = 5*1024*1024
    # depth camera    
    chunk_rm_depth = 1024
    # microphone
    chunk_mc = 512    
    profile_mc = hl2ss.AudioProfile.AAC_24000
    # spatial input
    chunk_si = 1024

    camera_pv = BufferedCamera(rx_pv, (host, chunk_pv, hl2ss.StreamMode.MODE_1, width_pv, height_pv, framerate_pv, profile_pv, bitrate_pv), 2048)
    camera_rm_depth = BufferedCamera(rx_rm_depth, (host, chunk_rm_depth, hl2ss.StreamMode.MODE_1), 512)
    camera_mc = BufferedCamera(rx_mc, (host, chunk_mc, profile_mc), 4096)
    camera_si = BufferedCamera(rx_si, (host, chunk_si), 4096)

    frame_stamp_pv = 0
    frame_stamp_rm_depth = 0
    frame_stamp_mc = 0
    frame_stamp_si = 0

    camera_pv.start_camera()
    camera_rm_depth.start_camera()
    camera_mc.start_camera()
    camera_si.start_camera()

    while True:
        if (camera_pv.frames_available(frame_stamp_pv)):
            data = camera_pv.get_buffered_frame(frame_stamp_pv)
            frame_stamp_pv += 1
            if (frame_stamp_pv == 30*10): # stop after 10 seconds to test join
                break
            cv2.imshow('PV', data.payload)
            cv2.waitKey(1)

        if (camera_rm_depth.frames_available(frame_stamp_rm_depth)):
            data = camera_rm_depth.get_buffered_frame(frame_stamp_rm_depth)
            frame_stamp_rm_depth += 1
            cv2.imshow('AB', data.payload.ab*8)
            cv2.imshow('Depth', data.payload.depth*8)
            cv2.waitKey(1)

        if (camera_mc.frames_available(frame_stamp_mc)):
            data = camera_mc.get_buffered_frame(frame_stamp_mc)
            frame_stamp_mc += 1

        if (camera_si.frames_available(frame_stamp_si)):
            data = camera_si.get_buffered_frame(frame_stamp_si)
            frame_stamp_si += 1
            si = hl2ss.unpacker_si(data.payload)
            if ((frame_stamp_si & 64) == 0):
                print('Detected => head : {h}, eye : {e}, left hand {l}, right hand {r}'.format(h=si.is_valid_head_pose(), e=si.is_valid_eye_ray(), l=si.is_valid_hand_left(), r=si.is_valid_hand_right()))
            
    camera_pv.stop_camera()
    camera_rm_depth.stop_camera()
    camera_mc.stop_camera()
    camera_si.stop_camera()


# entry point
if __name__ == '__main__':
    multiprocessing.freeze_support()
    viewer()

    

    

    

    


    # while (True):
    #     if (id == 0):
    #         lock_pv.acquire()
    #         din_pv.put(make_msg(IPC.CID_FRAMES_AVAILABLE, frame_stamp))
    #         event_pv.set()
    #         lock_pv.release()
    #         id = 1
    #     elif (id == 1):
    #         try:
    #             response = dout_pv.get_nowait()
    #             ok = True
    #         except:
    #             ok = False
    #         if (ok):
    #             if (response):
    #                 id = 2
    #             else:
    #                 id = 0
    #     elif (id == 2):
    #         lock_pv.acquire()
    #         din_pv.put(make_msg(IPC.CID_GET_BUFFERED_FRAME, frame_stamp))
    #         event_pv.set()
    #         lock_pv.release()
    #         frame_stamp += 1            
    #         id = 3
    #     elif (id == 3):
    #         try:
    #             response = dout_pv.get_nowait()
    #             ok = True
    #         except:
    #             ok = False
    #         if (ok):
    #             data_available = response is not None
    #             cv2.imshow('PV', response.payload)
    #             cv2.waitKey(1)
    #             id = 0


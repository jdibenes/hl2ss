
import requests
import json
import av
import io
import struct
import numpy as np
import collections
import cv2
import hl2ss


#------------------------------------------------------------------------------
# Network Client
#------------------------------------------------------------------------------

class _client:
    def open(self, host, port, chunk_size, user, password):
        self._response = requests.get(f'https://{host}/api/holographic/stream/{port}.mp4', auth=(user, password), verify=False, stream=True)
        self._iterator = self._response.iter_content(chunk_size)

    def recv(self):
        return next(self._iterator)

    def close(self):
        self._response.close()


#------------------------------------------------------------------------------
# Packet Unpacker
#------------------------------------------------------------------------------

class _box:
    def __init__(self, size, type, data):
        self.size = size
        self.type = type
        self.data = data


class _unpacker:
    def reset(self):
        self._buffer = bytearray()
        self._state  = 0

    def extend(self, data):
        self._buffer.extend(data)

    def unpack(self):
        length = len(self._buffer)

        while (True):
            if (self._state == 0):
                if (length >= 8):
                    self._box_l = struct.unpack('>I', self._buffer[0:4])[0]
                    self._box_t = self._buffer[4:8].decode()
                    self._state = 1
                    continue
            elif (self._state == 1):
                if (length >= self._box_l):
                    self._box_d  = self._buffer[8:self._box_l]
                    self._buffer = self._buffer[self._box_l:]
                    self._state  = 0
                    return True
            return False
        
    def get(self):
        return _box(self._box_l, self._box_t, self._box_d)


def flatten_box(box):
    subboxes = []
    offset = 0
    while (offset < len(box.data)):
        size = struct.unpack('>I', box.data[offset:(offset+4)])[0]
        subboxes.append(_box(size, box.data[(offset+4):(offset+8)].decode(), box.data[(offset+8):(offset+size)]))
        offset += size
    return subboxes


#------------------------------------------------------------------------------
# Packet Gatherer
#------------------------------------------------------------------------------

class _gatherer:
    def open(self, host, port, chunk_size, user, password):
        self._client = _client()
        self._unpacker = _unpacker()
        self._state = 0
        self._unpacker.reset()
        self._client.open(host, port, chunk_size, user, password)
        self._stsdindex = 0  

    def get_next_packet(self):
        packets = []
        while (True):
            self._unpacker.extend(self._client.recv())
            while (self._unpacker.unpack()):
                box = self._unpacker.get()
                if (self._state == 0):
                    if (box.type == 'moov'):
                        for moov_box in flatten_box(box):
                            if (moov_box.type == 'trak'):
                                for trak_box in flatten_box(moov_box):
                                    if (trak_box.type == 'mdia'):
                                        for mdia_box in flatten_box(trak_box):
                                            if (mdia_box.type == 'minf'):
                                                for minf_box in flatten_box(mdia_box):
                                                    if (minf_box.type == 'stbl'):
                                                        for stbl_box in flatten_box(minf_box):
                                                            if (stbl_box.type == 'stsd'):
                                                                stbl_data = stbl_box.data
                                                                if (stbl_data[12:16].decode() == 'avc1'):
                                                                    sps_data = stbl_data[106:134]
                                                                    pps_data = stbl_data[133:141]
                                                                    sps_data[0] = 0
                                                                    sps_data[1] = 0
                                                                    sps_data[2] = 0
                                                                    sps_data[3] = 1
                                                                    pps_data[0] = 0
                                                                    pps_data[1] = 0
                                                                    pps_data[2] = 0
                                                                    pps_data[3] = 1
                                                                    self._sps = sps_data
                                                                    self._pps = pps_data
                        self._state = 1
                elif (self._state == 1):
                    if (box.type == 'moof'):
                        streams = []
                        print('NEW MOOF')
                        for moof_box in flatten_box(box):
                            print(f'moof_box.type {moof_box.type}')
                            if (moof_box.type == 'traf'):
                                offset = 0
                                sample_sizes = []
                                for traf_box in flatten_box(moof_box):
                                    if (traf_box.type == 'tfhd'):
                                        id = struct.unpack('>I', traf_box.data[4:8])[0]
                                    elif (traf_box.type == 'trun'):
                                        sample_count = struct.unpack('>I', traf_box.data[4:8])[0]
                                        offset = struct.unpack('>i', traf_box.data[8:12])[0]
                                        sample_sizes = [struct.unpack('>I', traf_box.data[12+(16*i)+4:12+(16*i)+8])[0] for i in range(0, sample_count)]
                                streams.append((id, offset, sample_sizes))
                        self._state = 2
                elif (self._state == 2):
                    if (box.type == 'mdat'):
                        streams.append((-1, len(box.data), 0))
                        streams.sort(key=lambda stream : stream[1])
                        for i in range(0, len(streams) - 1):
                            stream_l = streams[i]
                            stream_h = streams[i+1]
                            packets.append((stream_l[0], box.data[stream_l[1]:stream_h[1]], stream_l[2]))
                        self._state = 1
                        return packets
    
    def close(self):
        self._client.close()

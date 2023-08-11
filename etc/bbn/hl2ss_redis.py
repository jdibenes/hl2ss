






'''
























import asyncio
import websockets.client











# start stop pv
    if (is_rs_host(host)):
        return
    
# connect
 if (is_rs_host(host)):
        c = _rs_gatherer()
        c.open(host, port, None)



#//////////////////////////////////////////////////////////////////////////////
# Extension: redis-streamer (NYU)
#//////////////////////////////////////////////////////////////////////////////

#------------------------------------------------------------------------------
# GOP Tagging
#------------------------------------------------------------------------------

class _extension_gop:
    def __init__(self, gop_size):
        self.aliased_index = 0
        self.gop_size = gop_size

    def extend(self, data):
        data.extend(struct.pack('<B', self.aliased_index))
        self.aliased_index = (self.aliased_index + 1) % self.gop_size


#------------------------------------------------------------------------------
# API redis-streamer
#------------------------------------------------------------------------------

def is_rs_host(host):
    return ':' in host


def _rs_get_stream_url_push(host, port):
    return f'ws://{host}/data/{get_port_name(port)}/push?header=0'


def _rs_get_stream_url_pull(host, port):
    return f'ws://{host}/data/{get_port_name(port)}/pull?header=0'


#------------------------------------------------------------------------------
# Network Client (Websockets)
#------------------------------------------------------------------------------

class _rs_client:
    def open(self, host, port, max_size):
        try:
            self._loop = asyncio.get_event_loop()
        except:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
        self._client = self._loop.run_until_complete(websockets.client.connect(_rs_get_stream_url_pull(host, port), max_size=max_size, compression=None))

    def recv(self):
        while (True):
            data = self._loop.run_until_complete(self._client.recv())
            if (len(data) > 0):
                return data

    def close(self):
        self._loop.run_until_complete(self._client.close())


#------------------------------------------------------------------------------
# Packet Gatherer (Websockets)
#------------------------------------------------------------------------------

class _rs_gatherer:
    def open(self, host, port, max_size):
        self._genlock = False
        self._client = _rs_client()
        self._client.open(host, port, max_size)

    def _fetch(self):
        data = self._client.recv()
        raw_packet = data[:-1]
        aliased_index = struct.unpack('<B', data[-1:])[0]
        return (aliased_index, raw_packet)
    
    def get_next_packet(self):
        aliased_index, data = self._fetch()
        while (not self._genlock):
            if (aliased_index == 0): 
                self._genlock = True
            else:
                aliased_index, data = self._fetch()
        return unpack_packet(data)
    
    def close(self):
        self._client.close()

        

        def configure_rm_vlc(self, decoded, host, port, chunk, mode, divisor, profile, gop_size, bitrate):
        self.configure(port, hl2ss.rx_decoded_rm_vlc(host, port, chunk, mode, divisor, profile, gop_size, bitrate) if (decoded) else hl2ss.rx_rm_vlc(host, port, chunk, mode, divisor, profile, gop_size, bitrate))

    def configure_rm_depth_ahat(self, decoded, host, port, chunk, mode, divisor, profile, gop_size, bitrate):
        self.configure(port, hl2ss.rx_decoded_rm_depth_ahat(host, port, chunk, mode, divisor, profile, gop_size, bitrate) if (decoded) else hl2ss.rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile, gop_size, bitrate))
        
    def configure_rm_depth_longthrow(self, decoded, host, port, chunk, mode, divisor, png_filter):
        self.configure(port, hl2ss.rx_decoded_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter) if (decoded) else hl2ss.rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter))

    def configure_rm_imu(self, host, port, chunk, mode):
        self.configure(port, hl2ss.rx_rm_imu(host, port, chunk, mode))

    def configure_pv(self, decoded, host, port, chunk, mode, width, height, framerate, divisor, profile, gop_size, bitrate, decoded_format):
        self.configure(port, hl2ss.rx_decoded_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, gop_size, bitrate, decoded_format) if (decoded) else hl2ss.rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, gop_size, bitrate))

    def configure_microphone(self, decoded, host, port, chunk, profile, level):
        self.configure(port, hl2ss.rx_decoded_microphone(host, port, chunk, profile, level) if (decoded) else hl2ss.rx_microphone(host, port, chunk, profile, level))

    def configure_si(self, host, port, chunk):
        self.configure(port, hl2ss.rx_si(host, port, chunk))

    def configure_eet(self, host, port, chunk, fps):
        self.configure(port, hl2ss.rx_eet(host, port, chunk, fps))
'''
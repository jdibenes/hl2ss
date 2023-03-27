
import os
import time
import asyncio
import websockets.client
import hl2ss

# ----------------------------- Settings ------------------------------------- #

# HoloLens address
HL_HOST = os.getenv("HOLOLENS_URL") or "192.168.1.7"

# Data server login
API_HOST = os.getenv('API_URL') or 'localhost:8000'

# ----------------------------- Streaming Basics ----------------------------- #

class StreamUpload:
    def __init__(self, host=HL_HOST, api_url=API_HOST):
        self.host = host
        self.api_url = api_url

    def __call__(self):
        while True:
            asyncio.run(self.forward_async())
            time.sleep(3)

    def create_client(self):
        raise NotImplementedError

    async def forward_async(self):
        extension_gop = hl2ss._extension_gop(self.gop_size)
        
        #try:
        async with websockets.client.connect(hl2ss._rs_get_stream_url_push(self.api_url, self.port), close_timeout=10, compression=None) as ws:
            with self.create_client() as client:                
                while True: # TODO: STOP
                    data = hl2ss.pack_packet(client.get_next_packet())
                    extension_gop.extend(data)
                    await ws.send(bytes(data))
                    await asyncio.sleep(0)
        #except Exception as e:
        #    print(e)

# ------------------------------- Side Cameras ------------------------------- #

class rm_vlc_upload(StreamUpload):
    mode     = hl2ss.StreamMode.MODE_1  # TODO: Config
    profile  = hl2ss.VideoProfile.H264_MAIN  # TODO: Config
    bitrate  = 3 * 1024 * 1024  # TODO: Config
    gop_size = hl2ss.get_gop_size(profile, hl2ss.Parameters_RM_VLC.FPS)
    
    def create_client(self):        
        return hl2ss.rx_rm_vlc(self.host, self.port, hl2ss.ChunkSize.RM_VLC, self.mode, self.profile, self.bitrate)
    
class rm_vlc_leftfront_upload(rm_vlc_upload):
    port = hl2ss.StreamPort.RM_VLC_LEFTFRONT

class rm_vlc_leftleft_upload(rm_vlc_upload):
    port = hl2ss.StreamPort.RM_VLC_LEFTLEFT

class rm_vlc_rightfront_upload(rm_vlc_upload):
    port = hl2ss.StreamPort.RM_VLC_RIGHTFRONT

class rm_vlc_rightright_upload(rm_vlc_upload):
    port = hl2ss.StreamPort.RM_VLC_RIGHTRIGHT

# ----------------------------------- Depth ---------------------------------- #

class rm_depth_ahat_upload(StreamUpload):
    port     = hl2ss.StreamPort.RM_DEPTH_AHAT
    mode     = hl2ss.StreamMode.MODE_1  # TODO: Config
    profile  = hl2ss.VideoProfile.H264_MAIN  # TODO: Config
    bitrate  = 8 * 1024 * 1024  # TODO: Config
    gop_size = hl2ss.get_gop_size(profile, hl2ss.Parameters_RM_DEPTH_AHAT.FPS)

    def create_client(self):
        return hl2ss.rx_rm_depth_ahat(self.host, self.port, hl2ss.ChunkSize.RM_DEPTH_AHAT, self.mode, self.profile, self.bitrate)

class rm_depth_longthrow_upload(StreamUpload):
    port       = hl2ss.StreamPort.RM_DEPTH_LONGTHROW
    mode       = hl2ss.StreamMode.MODE_1  # TODO: Config
    png_filter = hl2ss.PngFilterMode.Paeth  # TODO: Config
    gop_size   = 1

    def create_client(self):
        return hl2ss.rx_rm_depth_longthrow(self.host, self.port, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, self.mode, self.png_filter)

# ------------------------------------ IMU ----------------------------------- #

class rm_imu_upload(StreamUpload):
    mode     = hl2ss.StreamMode.MODE_1  # TODO: Config
    gop_size = 1

    def create_client(self):        
        return hl2ss.rx_rm_imu(self.host, self.port, self.chunk_size, self.mode)

class rm_imu_accelerometer_upload(rm_imu_upload):
    port       = hl2ss.StreamPort.RM_IMU_ACCELEROMETER
    chunk_size = hl2ss.ChunkSize.RM_IMU_ACCELEROMETER

class rm_imu_gyroscope_upload(rm_imu_upload):
    port       = hl2ss.StreamPort.RM_IMU_GYROSCOPE
    chunk_size = hl2ss.ChunkSize.RM_IMU_GYROSCOPE

class rm_imu_magnetometer_upload(rm_imu_upload):
    port       = hl2ss.StreamPort.RM_IMU_MAGNETOMETER
    chunk_size = hl2ss.ChunkSize.RM_IMU_MAGNETOMETER

# -------------------------------- Main Camera ------------------------------- #

class personal_video_upload(StreamUpload):
    port    = hl2ss.StreamPort.PERSONAL_VIDEO
    mode    = hl2ss.StreamMode.MODE_1 # TODO: Config
    #bitrate = 7 * 1024 * 1024 # TODO: Config
    profile = hl2ss.VideoProfile.H264_MAIN # TODO: Config

    def __init__(self, *a, width=760, height=428, fps=15, **kw):
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = int((width*height*fps*12)/50)
        self.gop_size = hl2ss.get_gop_size(self.profile, self.fps)
        super().__init__(*a, **kw)

    def create_client(self):
        hl2ss.start_subsystem_pv(self.host, self.port)
        return hl2ss.rx_pv(self.host, self.port, hl2ss.ChunkSize.PERSONAL_VIDEO, self.mode, self.width, self.height, self.fps, self.profile, self.bitrate)

# -------------------------------- Microphone -------------------------------- #

class microphone_upload(StreamUpload):
    port     = hl2ss.StreamPort.MICROPHONE
    profile  = hl2ss.AudioProfile.AAC_24000 # TODO: Config
    gop_size = 1

    def create_client(self):
        return hl2ss.rx_microphone(self.host, self.port, hl2ss.ChunkSize.MICROPHONE, self.profile)

# -------------------------------- Spatial Input ----------------------------- #

class spatial_input_upload(StreamUpload):
    port     = hl2ss.StreamPort.SPATIAL_INPUT
    gop_size = 1

    def create_client(self):
        return hl2ss.rx_si(self.host, self.port, hl2ss.ChunkSize.SPATIAL_INPUT)

# ---------------------------------------------------------------------------- #

# HL2SS IPC ports not supported
# PV + RM Depth AHAT not working in current versions of the HoloLens OS
# RM Depth AHAT + RM Depth Longthrow not supported by the Research Mode API
port_manifest = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
    hl2ss.StreamPort.RM_DEPTH_AHAT,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
    hl2ss.StreamPort.RM_IMU_GYROSCOPE,
    hl2ss.StreamPort.RM_IMU_MAGNETOMETER,
    hl2ss.StreamPort.PERSONAL_VIDEO,
    hl2ss.StreamPort.MICROPHONE,
    hl2ss.StreamPort.SPATIAL_INPUT
]

if __name__ == '__main__':
    import fire
    fire.Fire({ hl2ss.get_port_name(port) : globals()[hl2ss.get_port_name(port) + '_upload'] for port in port_manifest})


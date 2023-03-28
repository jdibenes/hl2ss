
import time
import hl2ss
import hl2ss_io

host = '192.168.1.7'
port = hl2ss.StreamPort.RM_VLC_RIGHTFRONT

if __name__ == "__main__":
    rx = hl2ss.rx_rm_vlc(host, port, hl2ss.ChunkSize.RM_VLC, hl2ss.StreamMode.MODE_1, hl2ss.VideoProfile.H264_HIGH, 1*1024*1024)
    wr = hl2ss_io.wr_process_rx(f'./data/{hl2ss.get_port_name(port)}.bin', rx, 'SINGLERX'.encode())
    wr.start()
    time.sleep(20)
    wr.stop()
    wr.join()

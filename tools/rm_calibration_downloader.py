
import argparse
import hl2ss
import hl2ss_3dcv

parser = argparse.ArgumentParser(description='HL2SS RM Calibration Downloader Tool.')
parser.add_argument('--host', help='HL2 IP address (e.g. 192.168.1.0)', required=True)
parser.add_argument('--path', help='HL2 calibration folder (e.g. ../calibration/)', required=True)

args = parser.parse_args()

host = args.host
path = args.path

ports = [hl2ss.StreamPort.RM_VLC_LEFTFRONT,
         hl2ss.StreamPort.RM_VLC_LEFTLEFT,
         hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
         hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
         hl2ss.StreamPort.RM_DEPTH_AHAT,
         hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
         hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
         hl2ss.StreamPort.RM_IMU_GYROSCOPE]

for port in ports:
    print('Fetching calibration for ' + hl2ss.get_port_name(port))
    hl2ss_3dcv.get_calibration_rm(host, port, path)

print('RM calibrations saved to ' + path)

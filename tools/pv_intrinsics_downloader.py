
import argparse
import hl2ss
import hl2ss_3dcv

parser = argparse.ArgumentParser(description='HL2SS PV Intrinsics Downloader Tool.')
parser.add_argument('--host', help='HL2 IP address (e.g. 192.168.1.0)', required=True)
parser.add_argument('--focus', help='Focus value in range [{fmin}, {fmax}]'.format(fmin=hl2ss.FocusValue.Min, fmax=hl2ss.FocusValue.Max), required=True)
parser.add_argument('--width', help='Video width in pixels', required=True)
parser.add_argument('--height', help='Video height in pixels', required=True)
parser.add_argument('--fps', help='Video framerate', required=True)
parser.add_argument('--path', help='HL2 calibration folder (e.g. ../calibration/)', required=True)

args = parser.parse_args()

host      = args.host
focus     = int(args.focus)
width     = int(args.width)
height    = int(args.height)
framerate = int(args.fps)
path      = args.path

pv_profile = hl2ss.VideoProfile.H264_BASE
pv_bitrate = 5*1024*1024

print('Setting PV focus to {focus}'.format(focus=focus))
hl2ss_3dcv.pv_optimize_for_cv(host, focus, hl2ss.ExposureMode.Auto, hl2ss.ExposureValue.Min, hl2ss.ColorTemperaturePreset.Auto)
print('Fetching PV calibration for {width}x{height}@{framerate}'.format(width=width, height=height, framerate=framerate))
calibration = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, path, focus, width, height, framerate, pv_profile, pv_bitrate, False)
print('PV calibration saved to ' + path)
print('PV intrinsics:')
print(calibration.intrinsics)

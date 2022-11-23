
import argparse
import hl2ss
import hl2ss_3dcv

parser = argparse.ArgumentParser(description='HL2SS PV Intrinsics Downloader Tool.')
parser.add_argument('--host', help='HL2 IP address (e.g. 192.168.1.0)', required=True)
parser.add_argument('--focus', help='Focus value in range [{fmin}, {fmax}]'.format(fmin=hl2ss.PV_FocusValue.Min, fmax=hl2ss.PV_FocusValue.Max), required=True)
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

if ((focus < hl2ss.PV_FocusValue.Min) or (focus > hl2ss.PV_FocusValue.Max)):
    print('Error: Focus value must be in range [{fmin}, {fmax}]'.format(fmin=hl2ss.PV_FocusValue.Min, fmax=hl2ss.PV_FocusValue.Max))
    quit()

client_rc = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)

hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
client_rc.wait_for_pv_subsystem(True)

print('Setting PV focus to {focus}'.format(focus=focus))
hl2ss_3dcv.pv_optimize_for_cv(host, focus, hl2ss.PV_ExposureMode.Auto, hl2ss.PV_ExposureValue.Min, hl2ss.PV_IsoSpeedMode.Auto, hl2ss.PV_IsoSpeedValue.Min, hl2ss.PV_ColorTemperaturePreset.Auto)
print('Fetching PV calibration for {width}x{height}@{framerate}'.format(width=width, height=height, framerate=framerate))
calibration = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, path, focus, width, height, framerate, False)
print('PV calibration saved to ' + path)
print('PV intrinsics:')
print(calibration.intrinsics)

hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
client_rc.wait_for_pv_subsystem(False)

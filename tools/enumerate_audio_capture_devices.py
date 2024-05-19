
import os
import json
import argparse
import hl2ss
import hl2ss_lnm

parser = argparse.ArgumentParser(description='HL2SS Enumerate Audio Capture Devices Tool.')
parser.add_argument('--host', help='HL2 IP address (e.g. 192.168.1.0)', required=True)
parser.add_argument('--path', help='Output folder (e.g. ../calibration/)', required=True)

args = parser.parse_args()

host = args.host
out  = args.path

filename = os.path.join(out, 'hl2_extended_audio_devices.json')

print('Downloading list...')
media_groups = json.loads(hl2ss_lnm.download_devicelist_extended_audio(host, hl2ss.StreamPort.EXTENDED_AUDIO))
print('Done')

with open(filename, 'wt') as f:
    f.write(json.dumps(media_groups, indent=2))

print(f'Wrote {filename}')

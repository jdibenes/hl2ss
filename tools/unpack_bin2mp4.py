
import argparse
import hl2ss_utilities

parser = argparse.ArgumentParser(description='HL2SS bin2mp4 Tool. Unpacks data recorded with hl2ss_io into a mp4 file.')
parser.add_argument('-I', '--input', action='append', required=True, help='Input bin files containing encoded (h264, hevc, or aac only) stream data (e.g., -I ./data/personal_video.bin -I ./data/microphone.bin)')
parser.add_argument('-O', '--output', required=True, help='Output file (e.g., ./data/rgb_video_with_sound.mp4)')
args = parser.parse_args()

hl2ss_utilities.unpack_to_mp4(args.input, args.output)

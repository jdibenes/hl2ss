#------------------------------------------------------------------------------
# Convert to MP4 example. Converts PV and Microphone data that was previously
# recorded using simple recorder to a single MP4 file.
#------------------------------------------------------------------------------

import os
import hl2ss
import hl2ss_ds

# Settings --------------------------------------------------------------------

# Directory containing the recorded data
path = './data/DS-2025-04-11-19-45-11'

# Name of output MP4 file
output_filename = 'pv_audio.mp4'

#------------------------------------------------------------------------------

filename_pv  = os.path.join(path, f'{hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)}.bin')
filename_mc  = os.path.join(path, f'{hl2ss.get_port_name(hl2ss.StreamPort.MICROPHONE)}.bin')
filename_mp4 = os.path.join(path, output_filename)

# Only bin files of H264/H265 video (except RM Depth AHAT) and AAC audio are supported
hl2ss_ds.unpack_to_mp4([filename_pv, filename_mc], filename_mp4)

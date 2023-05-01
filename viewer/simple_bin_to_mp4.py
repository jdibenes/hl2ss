
# MP4 conversion example
# Only RM VLC, RM Depth AHAT, PV, and Microphone are supported

import hl2ss_utilities

filenames_in = [
    './data/personal_video.bin',
    './data/rm_vlc_leftfront.bin',
    './data/microphone.bin'
]

filename_out = './data/video.mp4'

hl2ss_utilities.unpack_to_mp4(filenames_in, filename_out)

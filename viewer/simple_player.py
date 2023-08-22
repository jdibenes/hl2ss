#------------------------------------------------------------------------------
# Player example. Plays PV data recorded using simple recorder.
#------------------------------------------------------------------------------

import cv2
import os
import hl2ss_imshow
import hl2ss
import hl2ss_io

# Settings --------------------------------------------------------------------

# Directory containing the recorded data
path = './data'

#------------------------------------------------------------------------------

# Open PV bin file ------------------------------------------------------------
port = hl2ss.StreamPort.PERSONAL_VIDEO
filename = os.path.join(path, f'{hl2ss.get_port_name(port)}.bin')

# Stream type is detected automatically
# The format parameter is only used for PV and is ignored for other types
reader = hl2ss_io.create_rd(filename, hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')
reader.open()

# Display decoded contents ----------------------------------------------------
while (True):
    data = reader.get_next_packet() # get_next_packet() equivalent
    if (data is None):
        break # End of file
    
    # Same data structure as receiver (this is true for all stream types)
    print(f'Pose at time {data.timestamp}')
    print(data.pose)
    print(f'Focal length: {data.payload.focal_length}')
    print(f'Principal point: {data.payload.principal_point}')

    cv2.imshow('Video', data.payload.image)
    cv2.waitKey(1)

# Close PV bin file -----------------------------------------------------------
reader.close()

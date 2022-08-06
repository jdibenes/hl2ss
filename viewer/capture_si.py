#------------------------------------------------------------------------------
# si capture test
#------------------------------------------------------------------------------

import hl2ss
import os

# Settings --------------------------------------------------------------------

host = "192.168.1.15"
port = hl2ss.StreamPort.SPATIAL_INPUT
chunk_size = 1024
frames = 300
path = os.path.join('.', 'data')

#------------------------------------------------------------------------------

wr_frames = 0
rd_frames = 0

wr = hl2ss.wr_si(path)
rx = hl2ss.rx_si(host, port, chunk_size)

wr.open()
rx.open()

while (wr_frames < frames):
    data = rx.get_next_packet()
    wr.write(data)
    wr_frames += 1

rx.close()
wr.close()

rd = hl2ss.rd_si(path, chunk_size)
rd.open()

while (True):
    data = rd.read()
    if (data is None):
        break

    si = hl2ss.unpack_si(data.payload)
    print('Tracking status at time {ts}: Head {h}, Eye {e}, Left {l}, Right {r}'.format(ts=data.timestamp, h=si.is_valid_head_pose(), e=si.is_valid_eye_ray(), l=si.is_valid_hand_left(), r=si.is_valid_hand_right()))

    rd_frames += 1

rd.close()

print('written frames {v}'.format(v=wr_frames))
print('read frames {v}'.format(v=rd_frames))

#------------------------------------------------------------------------------
# rm imu capture test
#------------------------------------------------------------------------------

import hl2ss
import os

# Settings --------------------------------------------------------------------

host = "192.168.1.15"
port = hl2ss.StreamPort.RM_IMU_ACCELEROMETER
chunk_size = 1024
name = 'acc'
mode = hl2ss.StreamMode.MODE_1
frames = 100
path = os.path.join('.', 'data')

#------------------------------------------------------------------------------

wr_frames = 0
rd_frames = 0

wr = hl2ss.wr_rm_imu(path, name, mode)
rx = hl2ss.rx_rm_imu(host, port, chunk_size, mode)

wr.open()
rx.open()

while (wr_frames < frames):
    data = rx.get_next_packet()
    wr.write(data)
    wr_frames += 1
    
rx.close()
wr.close()

rd = hl2ss.rd_rm_imu(path, name, chunk_size)
rd.open()

while (True):
    data = rd.read()
    if (data is None):
        break

    print(data.pose)

    imu_data = hl2ss.unpack_rm_imu(data.payload)
    sample = imu_data.get_sample(0)
    print('Got {count} samples at time {ts}, first sample is (ticks = {st}, x = {x}, y = {y}, z = {z})'.format(count=imu_data.get_count(), ts=data.timestamp, st=sample.sensor_ticks_ns, x=sample.x, y=sample.y, z=sample.z))

    rd_frames += 1

rd.close()

print('written frames {v}'.format(v=wr_frames))
print('read frames {v}'.format(v=rd_frames))

#------------------------------------------------------------------------------
# This script receives extended eye tracking data from the HoloLens.
# See https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/extended-eye-tracking-native
#------------------------------------------------------------------------------

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Port
port = hl2ss.StreamPort.EXTENDED_EYE_TRACKER

# Target Frame Rate
# Options: 30, 60, 90
fps = 90

#------------------------------------------------------------------------------

client = hl2ss.rx_eet(host, port, hl2ss.ChunkSize.SPATIAL_INPUT, fps)
client.open()

data = client.get_next_packet()
data.payload = hl2ss.unpack_eet(data.payload)

print(data.timestamp)
print(data.payload._reserved)

print(data.payload.combined_ray.origin)
print(data.payload.combined_ray.direction)
print(data.payload.left_ray.origin)
print(data.payload.left_ray.direction)
print(data.payload.right_ray.origin)
print(data.payload.right_ray.direction)
print(data.payload.left_openness)
print(data.payload.right_openness)
print(data.payload.vergence_distance)

print(data.payload.calibration_valid)
print(data.payload.combined_ray_valid)
print(data.payload.left_ray_valid)
print(data.payload.right_ray_valid)
print(data.payload.left_openness_valid)
print(data.payload.right_openness_valid)
print(data.payload.vergence_distance_valid)
print(data.pose)

client.close()

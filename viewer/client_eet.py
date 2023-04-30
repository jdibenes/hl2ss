#------------------------------------------------------------------------------
# This script receives extended eye tracking data from the HoloLens.
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

client = hl2ss.rx_eet(host, port, hl2ss.ChunkSize.EXTENDED_EYE_TRACKER, fps)
client.open()

data = client.get_next_packet()
data.payload = hl2ss.unpack_eet(data.payload)

# See
# https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/extended-eye-tracking-native
# for details

print(f'Tracking status at time {data.timestamp}')
print('Pose')
print(data.pose)
print(f'Calibration valid: {data.payload.calibration_valid}')
print(f'Combined eye gaze: Valid={data.payload.combined_ray_valid} Origin={data.payload.combined_ray.origin} Direction={data.payload.combined_ray.direction}')
print(f'Left eye gaze: Valid={data.payload.left_ray_valid} Origin={data.payload.left_ray.origin} Direction={data.payload.left_ray.direction}')
print(f'Right eye gaze: Valid={data.payload.right_ray_valid} Origin={data.payload.right_ray.origin} Direction={data.payload.right_ray.direction}')

# "...not supported by HoloLens 2 at this time"
print(f'Left eye openness: Valid={data.payload.left_openness_valid} Value={data.payload.left_openness}')
print(f'Right eye openness: Valid={data.payload.right_openness_valid} Value={data.payload.right_openness}')
print(f'Vergence distance: Valid={data.payload.vergence_distance_valid} Value={data.payload.vergence_distance}')

client.close()

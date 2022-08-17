#------------------------------------------------------------------------------
# This script enables/disables the display marker on the HoloLens used to show
# the user the lower boundary of the field of view of the front camera.
#------------------------------------------------------------------------------

import hl2ss
import struct

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Marker enable/disable
# 0: disable
# 1: enable
marker_state = 1

#------------------------------------------------------------------------------

client = hl2ss.client()

client.open(host, hl2ss.StreamPort.REMOTE_CONFIGURATION)
client.sendall(struct.pack('<B', marker_state))
client.close()

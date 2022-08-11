
import hl2ss
import struct

marker_state = 0

client = hl2ss.client()

client.open('192.168.1.15', hl2ss.StreamPort.REMOTE_CONFIGURATION)
client.sendall(struct.pack('<B', marker_state))
client.close()

#------------------------------------------------------------------------------
# This script demonstrates how to send a command with a string parameter to the
# Unity app. The command handler must be implemented in the hl2ss.cs script, in
# the ProcessMessage method for the corresponding command id.
#------------------------------------------------------------------------------

import struct
import hl2ss
import rus

#------------------------------------------------------------------------------

# HL2 address
host = '192.168.1.7'

# text to send to the Unity app
text = 'hello from python!!'

#------------------------------------------------------------------------------

client = hl2ss._client() # create hl2ss client object
client.open(host, rus.Port.IPC) # connect to HL2

data = text.encode('utf-8') # encode string as utf8

# create command string
# - command id (4 byte integer)
# - parameter length (4 byte integer)
# - parameter (parameter length bytes) 
command = bytearray()
command.extend(struct.pack('<II', 21, len(data)))
command.extend(data)

client.sendall(command) # send command string to Unity app
response = client.download(4, hl2ss.ChunkSize.SINGLE_TRANSFER) # receive response from Unity app (4 byte integer)

client.close() # disconnect

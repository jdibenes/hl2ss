#------------------------------------------------------------------------------
# This script sends a string to the Unity app running on the HoloLens which is
# then spoken using TTS.
#------------------------------------------------------------------------------

import hl2ss
import hl2ss_lnm
import hl2ss_rus

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Text
text = 'Hello from Python!'

#------------------------------------------------------------------------------

ipc = hl2ss_lnm.ipc_umq(host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE)
ipc.open()

key = 0

display_list = hl2ss_rus.command_buffer()
display_list.say(text)
ipc.push(display_list) # Send command to server
results = ipc.pull(display_list) # Get result from server
print(f'Response: {results[0]}')

ipc.close()

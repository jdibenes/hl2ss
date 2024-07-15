#------------------------------------------------------------------------------
# This script demonstrates how to receive a command with a string parameter 
# from a Unity app using the plugin. The command handler for this client is in
# the MQXSkeleton.cs script.
#------------------------------------------------------------------------------

import hl2ss
import hl2ss_lnm
import time

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

#------------------------------------------------------------------------------

client = hl2ss_lnm.ipc_gmq(host, hl2ss.IPCPort.GUEST_MESSAGE_QUEUE) # Create hl2ss client object
client.open() # Connect to HL2

# Main Loop: receive just one message
while (True):
    msg = client.pull() # Try to download message
    if (msg is not None): # Message received from Unity app
        client.push(1) # Send single uint32 response to Unity app (0xFFFFFFFF reserved)
        break # Stop loop
    time.sleep(0.1) # No messages available, wait...

client.close() # Disconnect

command_id     = msg[0] # Command id (uint32) user defined (0xFFFFFFFF reserved)
command_params = msg[1] # Command params (bytes) user defined

if (command_id == 0xFFFFFFFE):
    # Debug message
    # Command id: 0xFFFFFFFE
    # Command params: utf-8 string
    text = command_params.decode('utf-8') # Convert bytes to string
    print(text) # Print string
else:
    # Unknown message
    print(f'Received command id={command_id} with params={command_params.hex()}') # Print command


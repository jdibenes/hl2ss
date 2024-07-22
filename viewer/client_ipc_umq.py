#------------------------------------------------------------------------------
# This script demonstrates how to send a command with a string parameter to a
# Unity app using the plugin. The command handler for this client is in the
# IPCSkeleton.cs script.
#------------------------------------------------------------------------------

import hl2ss
import hl2ss_lnm

#------------------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Text to send
text = 'Hello from python!'

#------------------------------------------------------------------------------

class command_buffer(hl2ss.umq_command_buffer):
    # Command structure
    # id:     u32 (4 bytes)
    # size:   u32 (4 bytes)
    # params: size bytes

    # Send string to Visual Studio debugger
    def debug_message(self, text):
        # Command id: 0xFFFFFFFE
        # Command params: string encoded as utf-8
        self.add(0xFFFFFFFE, text.encode('utf-8')) # Use the add method from hl2ss.umq_command_buffer to pack commands

    # See hl2ss_rus.py and the unity_sample scripts for more examples.


client = hl2ss_lnm.ipc_umq(host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE) # Create hl2ss client object
client.open() # Connect to HL2

buffer = command_buffer() # Create command buffer
buffer.debug_message(text) # Append send_debug_message command

client.push(buffer) # Send commands in buffer to the Unity app
response = client.pull(buffer) # Receive response from the Unity app (4 byte integer per command)

print(f'response={response}')

client.close() # Disconnect

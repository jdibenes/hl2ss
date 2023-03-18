#------------------------------------------------------------------------------
# This script demonstrates how to send a command with a string parameter to the
# Unity app. The command handler must be implemented in the hl2ss.cs script, in
# the ProcessMessage method for the corresponding command id.
#------------------------------------------------------------------------------

import hl2ss

#------------------------------------------------------------------------------

# HL2 address
host = '192.168.1.7'

# text to send to the Unity app
text = 'hello from python!!'

#------------------------------------------------------------------------------

class command_buffer(hl2ss.umq_command_buffer):
    def send_text(self, text):
        # create command string
        # command id (4 byte integer)
        # data (bytes)
        self.add(21, text.encode('utf-8')) # assuming command 21 implemented in hl2ss.cs


client = hl2ss.ipc_umq(host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE) # create hl2ss client object
client.open() # connect to HL2

buffer = command_buffer() # create command buffer
buffer.send_text(text) # append send text command

client.push(buffer) # send command string to Unity app
response = client.pull(buffer) # receive response from Unity app (4 byte integer per command)

client.close() # disconnect

#------------------------------------------------------------------------------
# This script registers voice commands on the HoloLens and continously checks
# if any of the registered commands has been heard.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
import hl2ss_lnm
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Voice commands
strings = ['cat', 'dog', 'red', 'blue']

#------------------------------------------------------------------------------

listener = hl2ss_utilities.key_listener(keyboard.Key.esc)
listener.open()

client = hl2ss_lnm.ipc_vi(host, hl2ss.IPCPort.VOICE_INPUT)
client.open()

# See
# https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/voice-input-in-directx
# for details

client.start(strings)
print('Ready. Try saying any of the commands you defined.')

while (not listener.pressed()):
    events = client.pop()
    for event in events:
        # See
        # https://learn.microsoft.com/en-us/uwp/api/windows.media.speechrecognition.speechrecognitionresult?view=winrt-22621
        # for result details
        print(f'Event: Command={client.translate(event.index)} Index={event.index} Confidence={event.confidence} Duration={event.phrase_duration} Start={event.phrase_start_time} RawConfidence={event.raw_confidence}')

client.stop()
client.close()

listener.close()

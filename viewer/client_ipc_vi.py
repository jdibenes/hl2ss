#------------------------------------------------------------------------------
# This script registers voice commands on the HoloLens and continously checks
# if any of the registered commands has been heard.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import hl2ss
<<<<<<< HEAD:viewer/client_vi.py
import configparser
=======
import hl2ss_lnm
>>>>>>> 5d92301451f23c976ebcf6f65a35728896a2bb09:viewer/client_ipc_vi.py

# Settings --------------------------------------------------------------------
config = configparser.ConfigParser()
config.read('config.ini')
# HoloLens address
host = config['DEFAULT']['ip']

# Voice commands
strings = ['cat', 'dog', 'red', 'blue']

#------------------------------------------------------------------------------

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

def get_word(strings, index):
    if ((index < 0) or (index >= len(strings))):
        return '_UNKNOWN_'
    else:
        return strings[index]

listener = keyboard.Listener(on_press=on_press)
listener.start()

client = hl2ss_lnm.ipc_vi(host, hl2ss.IPCPort.VOICE_INPUT)
client.open()

# See
# https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/voice-input-in-directx
# for details

client.create_recognizer()
if (client.register_commands(True, strings)):
    print('Ready. Try saying any of the commands you defined.')
    client.start()    
    while (enable):
        events = client.pop()
        for event in events:
            event.unpack()
            # See
            # https://learn.microsoft.com/en-us/uwp/api/windows.media.speechrecognition.speechrecognitionresult?view=winrt-22621
            # for result details
            print(f'Event: Command={get_word(strings, event.index)} Index={event.index} Confidence={event.confidence} Duration={event.phrase_duration} Start={event.phrase_start_time} RawConfidence={event.raw_confidence}')
    client.stop()
    client.clear()

client.close()

listener.join()

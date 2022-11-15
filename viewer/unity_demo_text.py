#------------------------------------------------------------------------------
# This script adds a 3D TextMeshPro object to the Unity scene. Press esc to
# stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import rus
import threading

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Port
port = rus.Port.IPC

# Position in world space (x, y, z) in meters
position = [0, 0, 1]

# Rotation in world space (x, y, w, z) as a quaternion
rotation = [0, 0, 0, 1]

# Text
text = 'Hello from Python!'

# Font size
font_size = 0.4

# Text color
rgba = [1, 1, 1, 1]

#------------------------------------------------------------------------------

stop_event = threading.Event()

def on_press(key):
    if (key == keyboard.Key.esc):
        stop_event.set()
        return False
    return True

listener = keyboard.Listener(on_press=on_press)
listener.start()

ipc = rus.connect_client_mq(host, port)
key = 0

display_list = rus.create_command_buffer()
display_list.begin_display_list() # Begin command sequence
display_list.remove_all() # Remove all objects that were created remotely
display_list.create_text() # Create text object, server will return its id
display_list.set_target_mode(rus.TargetMode.UseLast) # Set server to use the last created object as target, this avoids waiting for the id of the text object
display_list.set_text(key, font_size, rgba, text) # Set text
display_list.set_world_transform(key, position, rotation, [1, 1, 1]) # Set the world transform of the text object
display_list.set_active(key, rus.ActiveState.Active) # Make the text object visible
display_list.set_target_mode(rus.TargetMode.UseID) # Restore target mode
display_list.end_display_list() # End command sequence
ipc.push(display_list) # Send commands to server
results = ipc.pop(display_list) # Get results from server
key = results[2] # Get the text object id, created by the 3rd command in the list

print('Created text object with id {iid}'.format(iid=key))

stop_event.wait()

command_buffer = rus.create_command_buffer()
command_buffer.remove(key) # Destroy text object
ipc.push(command_buffer)
results = ipc.pop(command_buffer)

ipc.close()

listener.join()

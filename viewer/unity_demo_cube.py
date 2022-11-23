#------------------------------------------------------------------------------
# This script adds a cube to the Unity scene and animates it. Press esc to
# stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import rus

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Port
port = rus.Port.IPC

# Initial position in world space (x, y, z) in meters
position = [0, 0, 0]

# Initial rotation in world space (x, y, z, w) as a quaternion
rotation = [0, 0, 0, 1]

# Initial scale in meters
scale = [0.2, 0.2, 0.2]

# Initial color
rgba = [1, 1, 1, 1]

#------------------------------------------------------------------------------

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

ipc = rus.connect_client_mq(host, port)
key = 0

display_list = rus.create_command_buffer()
display_list.begin_display_list() # Begin command sequence
display_list.remove_all() # Remove all objects that were created remotely
display_list.create_primitive(rus.PrimitiveType.Cube) # Create a cube, server will return its id
display_list.set_target_mode(rus.TargetMode.UseLast) # Set server to use the last created object as target, this avoids waiting for the id of the cube
display_list.set_world_transform(key, position, rotation, scale) # Set the world transform of the cube
display_list.set_color(key, rgba) # Set the color of the cube
display_list.set_active(key, rus.ActiveState.Active) # Make the cube visible
display_list.set_target_mode(rus.TargetMode.UseID) # Restore target mode
display_list.end_display_list() # End command sequence
ipc.push(display_list) # Send commands to server
results = ipc.pop(display_list) # Get results from server
key = results[2] # Get the cube id, created by the 3rd command in the list

print('Created cube with id {iid}'.format(iid=key))

z = 0
delta = 0.01

while (enable):
    z += delta

    if (z <= 0):
        z = 0
        delta = -delta
    elif (z >= 1):
        z = 1
        delta = -delta

    position[2] = z

    display_list = rus.create_command_buffer()
    display_list.begin_display_list()
    display_list.set_world_transform(key, position, rotation, scale)
    display_list.set_color(key, [z, 0, 1-z, 1-z]) # Semi-transparency is supported
    display_list.end_display_list()
    ipc.push(display_list)
    results = ipc.pop(display_list)

command_buffer = rus.create_command_buffer()
command_buffer.remove(key) # Destroy cube
ipc.push(command_buffer)
results = ipc.pop(command_buffer)

ipc.close()

listener.join()

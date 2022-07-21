
import keyboard
import rus

host = '192.168.1.15'
port = rus.IPCPort.MESSAGE_QUEUE

key = 0
position = [0, 0, 0]
rotation = [0, 0, 0, 1]
scale = [0.2, 0.2, 0.2]
rgba = [1, 1, 1, 1]

ipc = rus.connect_client_mq(host, port)

display_list = bytearray()
rus.begin_display_list(display_list)
rus.remove_all(display_list)
rus.create_primitive(display_list, rus.PrimitiveType.Cube)
rus.set_target_mode(display_list, 1)
rus.set_world_transform(display_list, key, position, rotation, scale)
rus.set_color(display_list, key, rgba)
rus.set_active(display_list, key, 1)
rus.set_target_mode(display_list, 0)
rus.end_display_list(display_list)
ipc.sendall(display_list)

results = rus.get_results(ipc, 9)
key = results[2]

z = 0
delta = 0.01

while True:
    z += delta

    if (z <= 0):
        z = 0
        delta = -delta
    elif (z >= 1):
        z = 1
        delta = -delta

    position[2] = z

    display_list = bytearray()
    rus.begin_display_list(display_list)
    rus.set_world_transform(display_list, key, position, rotation, scale)
    rus.set_color(display_list, key, [z, 0, 1-z, 1-z])
    rus.end_display_list(display_list)
    ipc.sendall(display_list)

    results = rus.get_results(ipc, 4)

    if (keyboard.is_pressed('space')):
        break

command_buffer = bytearray()
rus.remove(command_buffer, key)
ipc.sendall(command_buffer)

results = rus.get_results(ipc, 1)

ipc.close()


import keyboard
import rus

host = '192.168.1.15'
port = rus.IPCPort.MESSAGE_QUEUE

with open('set_texture_test.jpg', mode='rb') as jpg:
    texture = jpg.read()

key = 0
position = [int(1440/2), int(936/2), 40]
rotation = [0, 0, 0, 1]
scale = [256, 256, 1]

ipc = rus.connect_client_mq(host, port)

display_list = bytearray()
rus.begin_display_list(display_list)
rus.remove_all(display_list)
rus.create_primitive(display_list, rus.PrimitiveType.Cube)
rus.set_target_mode(display_list, 1)
rus.set_local_transform(display_list, key, position, rotation, scale)
rus.set_texture(display_list, key, texture)
rus.set_active(display_list, key, 1)
rus.set_target_mode(display_list, 0)
rus.end_display_list(display_list)
ipc.sendall(display_list)

results = rus.get_results(ipc, 9)
key = results[2]

while True:
    if (keyboard.is_pressed('space')):
        break

command_buffer = bytearray()
rus.remove(command_buffer, key)
ipc.sendall(command_buffer)

results = rus.get_results(ipc, 1)

ipc.close()

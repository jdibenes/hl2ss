
import keyboard
import rus

host = '192.168.1.15'
port = rus.IPCPort.MESSAGE_QUEUE

ipc = rus.connect_client_mq(host, port)
key = 0

display_list = bytearray()
rus.begin_display_list(display_list)
rus.remove_all(display_list)
rus.create_text(display_list)
rus.set_target_mode(display_list, 1)
rus.set_text(display_list, key, 0.4, [1, 1, 1, 1], 'Hello from Python!')
rus.set_world_transform(display_list, key, [0, 0, 1], [0, 0, 0, 1], [1, 1, 1])
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

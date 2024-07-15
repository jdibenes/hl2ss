
import hl2ss
import hl2ss_lnm
import time

host = '192.168.1.7'

client = hl2ss_lnm.ipc_gmq(host, hl2ss.IPCPort.GUEST_MESSAGE_QUEUE)
client.open()

while (True):
    msg = client.pull()
    if (msg is not None):
        client.push(1)
        break
    time.sleep(0.1)

client.close()

command = msg[0]
param = msg[1]

if (command == 0xFFFFFFFE):
    text = param.decode('utf-8')
    print(text)
else:
    print(f'Received command={command} with param={param.hex()}')



import hl2ss

client = hl2ss.ipc_rcx(hl2ss.IPCPort.REMOTE_CONFIGURATION)
client.open()

sources = dict()

for i in range(0, 60):
    if (client.select(1)):
        message = client.get_beacon()
        if (message is not None):
            sources[message[0]] = message[1:]
            print(message)

client.close()

print(sources)

#------------------------------------------------------------------------------
# This script downloads Scene Understanding data.
#------------------------------------------------------------------------------

import hl2ss

host = '192.168.1.7'
port = hl2ss.IPCPort.SCENE_UNDERSTANDING

kinds = 255 #hl2ss.SU_KindFlag.Floor | hl2ss.SU_KindFlag.Wall | hl2ss.SU_KindFlag.Platform | hl2ss.SU_KindFlag.Ceiling

client = hl2ss.ipc_su(host, port)
client.open()

task = hl2ss.su_task(True, True, False, True, hl2ss.SU_MeshLOD.Fine, 10.0, hl2ss.SU_Create.New,  kinds, True, True, True, True, False, False, [])
result = client.query(task)

client.close()

if (result is None):
    print('Scene query failed')
    quit()

result.unpack()

print('Extrinsics')
print(result.extrinsics)

print('Pose')
print(result.pose)

for item in result.items:
    print(f'SceneObject {item.id.hex()} {item.kind} {item.alignment} {item.extents} {item.orientation} {item.position}')

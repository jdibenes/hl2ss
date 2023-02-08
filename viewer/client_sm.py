
import hl2ss

host = '192.168.1.7'
port = hl2ss.IPCPort.SPATIAL_MAPPING

volumes = hl2ss.sm_volume()
volumes.add_box([0.0, 0.0, 0.0], [8.0, 8.0, 8.0])
tasks = hl2ss.sm_mesh_task(2)

client = hl2ss.ipc_sm(host, port)
client.open()

client.create_observer()

client.set_volumes(volumes)

ids = client.get_observed_surfaces()

tasks.add_task(ids[:16], 1000, hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized, hl2ss.SM_TriangleIndexFormat.R16UInt, hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized, True)
tasks.add_task(ids[16:32], 1000, hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized, hl2ss.SM_TriangleIndexFormat.R16UInt, hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized, True)
meshes = client.get_meshes(tasks)

client.close()

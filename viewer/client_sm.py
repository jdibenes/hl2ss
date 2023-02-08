
import numpy as np
import open3d as o3d
import hl2ss

host = '192.168.1.7'
port = hl2ss.IPCPort.SPATIAL_MAPPING

threads = 2
tpcm = 1000
vpf = hl2ss.SM_VertexPositionFormat.R32G32B32A32Float #hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized
tif = hl2ss.SM_TriangleIndexFormat.R32Uint #hl2ss.SM_TriangleIndexFormat.R16UInt
vnf = hl2ss.SM_VertexNormalFormat.R32G32B32A32Float #hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized

volumes = hl2ss.sm_bounding_volume()
volumes.add_box([0.0, 0.0, 0.0], [8.0, 8.0, 8.0])
tasks = hl2ss.sm_mesh_task()

client = hl2ss.ipc_sm(host, port)
client.open()
client.create_observer()
client.set_volumes(volumes)

ids = client.get_observed_surfaces()
for i in range(0, len(ids)):
    tasks.add_task(ids[i], tpcm, vpf, tif, vnf, True)

meshes = client.get_meshes(tasks, threads)
client.close()

tms = []
for mesh in meshes.values():
    if (mesh is None):
        continue

    mesh.unpack(vpf, tif, vnf)

    mesh.vertex_positions[:, 0:3] = mesh.vertex_positions[:, 0:3] * mesh.vertex_position_scale
    mesh.vertex_positions = mesh.vertex_positions @ mesh.pose
    mesh.vertex_normals = mesh.vertex_normals @ mesh.pose

    tm = o3d.geometry.TriangleMesh()
    tm.vertices = o3d.utility.Vector3dVector(mesh.vertex_positions[:, 0:3])
    tm.vertex_colors = o3d.utility.Vector3dVector(mesh.vertex_normals[:, 0:3].reshape((-1, 3)).astype(np.float64))
    tm.vertex_normals = o3d.utility.Vector3dVector(mesh.vertex_normals[:, 0:3].reshape((-1, 3)).astype(np.float64))
    tm.triangles = o3d.utility.Vector3iVector(mesh.triangle_indices)

    tms.append(tm)

o3d.visualization.draw_geometries(tms, mesh_show_back_face=True)

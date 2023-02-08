
import numpy as np
import open3d as o3d
import hl2ss

host = '192.168.1.7'
port = hl2ss.IPCPort.SPATIAL_MAPPING

tpcm = 1000
vpf = hl2ss.SM_VertexPositionFormat.R16G16B16A16IntNormalized
tif = hl2ss.SM_TriangleIndexFormat.R16UInt
vnf = hl2ss.SM_VertexNormalFormat.R8G8B8A8IntNormalized


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
#tasks.add_task(ids[0], tpcm, vpf, tif, vnf, True)
#tasks.add_task(ids[1], tpcm, vpf, tif, vnf, True)
meshes = client.get_meshes(tasks, 2)
tms = []

for i in range(0, len(meshes)):
    meshes[i].unpack(vpf, tif, vnf)

    vp = (meshes[i].vertex_positions[:, 0:3] / meshes[i].vertex_positions[:, 3].reshape((-1, 1))) * meshes[i].vertex_position_scale
    vn = meshes[i].vertex_normals[:, 0:3] / np.linalg.norm(meshes[i].vertex_normals[:, 0:3], axis=1).reshape((-1, 1))
#print(vp[:9])
#print(vn[:9])

    tm = o3d.geometry.TriangleMesh()
    tm.vertices = o3d.utility.Vector3dVector(vp)
    #tm.vertex_normals = o3d.utility.Vector3dVector(vn)
    tm.triangles = o3d.utility.Vector3iVector(meshes[i].triangle_indices)

    tms.append(tm)

print(len(tms))
o3d.visualization.draw_geometries(tms)




#print(meshes[0].vertex_positions.shape)
#print(meshes[0].triangle_indices.shape)
#print(meshes[0].vertex_normals.shape)

#print(meshes[0].vertex_position_scale)
#print(meshes[0].vertex_positions[0:12])
#print(meshes[0].triangle_indices[0:8])
#print(meshes[0].vertex_normals[0:12])


client.close()

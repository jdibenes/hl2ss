#------------------------------------------------------------------------------
# This script downloads Spatial Mapping data from the HoloLens and displays it.
#------------------------------------------------------------------------------

import open3d as o3d
import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Port
port = hl2ss.IPCPort.SPATIAL_MAPPING

# Maximum triangles per cubic meter
tpcm = 1000

# Data format
vpf = hl2ss.SM_VertexPositionFormat.R32G32B32A32Float
tif = hl2ss.SM_TriangleIndexFormat.R32Uint
vnf = hl2ss.SM_VertexNormalFormat.R32G32B32A32Float

# Include normals
normals = True

# Maximum number of active threads (on the HoloLens) to compute meshes
threads = 2

# Region of 3D space to sample (bounding box)
# All units are in meters
center  = [0.0, 0.0, 0.0] # Position of the box
extents = [8.0, 8.0, 8.0] # Dimensions of the box

#------------------------------------------------------------------------------

# Download meshes -------------------------------------------------------------
# See
# https://learn.microsoft.com/en-us/windows/mixed-reality/develop/native/spatial-mapping-in-directx
# for details

client = hl2ss.ipc_sm(host, port)

client.open()

client.create_observer()

volumes = hl2ss.sm_bounding_volume()
volumes.add_box(center, extents)
client.set_volumes(volumes)

ids = client.get_observed_surfaces()
tasks = hl2ss.sm_mesh_task()
for id in ids:
    tasks.add_task(id, tpcm, vpf, tif, vnf, normals)

meshes = client.get_meshes(tasks, threads)

client.close()

print(f'Observed {len(ids)} surfaces')

# Display meshes --------------------------------------------------------------

open3d_meshes = []

for index, mesh in meshes.items():
    id_hex = ids[index].hex()

    if (mesh is None):
        print(f'Task {index}: surface id {id_hex} compute mesh failed')
        continue

    mesh.unpack(vpf, tif, vnf)

    print(f'Task {index}: surface id {id_hex} has {mesh.vertex_positions.shape[0]} vertices {mesh.triangle_indices.shape[0]} triangles {mesh.vertex_normals.shape[0]} normals')

    mesh.vertex_positions[:, 0:3] = mesh.vertex_positions[:, 0:3] * mesh.vertex_position_scale
    mesh.vertex_positions = mesh.vertex_positions @ mesh.pose
    mesh.vertex_normals = mesh.vertex_normals @ mesh.pose

    open3d_mesh = o3d.geometry.TriangleMesh()
    open3d_mesh.vertices = o3d.utility.Vector3dVector(mesh.vertex_positions[:, 0:3])
    open3d_mesh.vertex_colors = o3d.utility.Vector3dVector(mesh.vertex_normals[:, 0:3])
    open3d_mesh.triangles = o3d.utility.Vector3iVector(mesh.triangle_indices)

    open3d_meshes.append(open3d_mesh)

o3d.visualization.draw_geometries(open3d_meshes, mesh_show_back_face=True)

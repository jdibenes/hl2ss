#------------------------------------------------------------------------------
# This script downloads Scene Understanding data from the HoloLens and displays
# it.
#------------------------------------------------------------------------------

import open3d as o3d
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_sa

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Query parameters
enable_scene_object_quads = True
enable_scene_object_meshes = True
enable_only_observed_scene_objects = False
enable_world_mesh = True
requested_mesh_level_of_detail = hl2ss.SU_MeshLOD.Fine
query_radius = 10.0 # Meters
create_mode = hl2ss.SU_Create.New
kinds = hl2ss.SU_KindFlag.Background | hl2ss.SU_KindFlag.Wall | hl2ss.SU_KindFlag.Floor | hl2ss.SU_KindFlag.Ceiling | hl2ss.SU_KindFlag.Platform | hl2ss.SU_KindFlag.Unknown | hl2ss.SU_KindFlag.World | hl2ss.SU_KindFlag.CompletelyInferred
get_orientation = True
get_position = True
get_location_matrix = True
get_quad = True
get_meshes = True
get_collider_meshes = True

# To track surfaces between scenes
# Create a new scene using SU_Create.NewFromPrevious and add the GUID of the 
# surface(s) of interest found in the previous scene
# If the surface is found in the new scene it will be returned
guid_list = [] 

#------------------------------------------------------------------------------

kind_color = {
    hl2ss.SU_Kind.Background         : [0, 0, 0],
    hl2ss.SU_Kind.Ceiling            : [0, 0, 1],
    hl2ss.SU_Kind.CompletelyInferred : [0, 1, 0],
    hl2ss.SU_Kind.Floor              : [0, 1, 1],
    hl2ss.SU_Kind.Platform           : [1, 0, 0],
    hl2ss.SU_Kind.Unknown            : [1, 0, 1],
    hl2ss.SU_Kind.Wall               : [1, 1, 0],
    hl2ss.SU_Kind.World              : [1, 1, 1],
}

# Download data ---------------------------------------------------------------
# See
# https://learn.microsoft.com/en-us/windows/mixed-reality/develop/unity/scene-understanding-sdk
# for details

task = hl2ss.su_task(enable_scene_object_quads,
                     enable_scene_object_meshes, 
                     enable_only_observed_scene_objects, 
                     enable_world_mesh, 
                     requested_mesh_level_of_detail, 
                     query_radius, 
                     create_mode, 
                     kinds,
                     get_orientation, 
                     get_position, 
                     get_location_matrix, 
                     get_quad,
                     get_meshes, 
                     get_collider_meshes, 
                     guid_list)
task.pack()

client = hl2ss_lnm.ipc_su(host, hl2ss.IPCPort.SCENE_UNDERSTANDING)
client.open()
result = client.query(task)
client.close()

# Display meshes --------------------------------------------------------------

result.unpack()

print('Extrinsics')
print(result.extrinsics)
print('Pose')
print(result.pose)
print(f'Items found: {len(result.items)}')

open3d_meshes = []
collider_meshes = []

for item in result.items:
    item.unpack()
    print(f'SceneObject ID={item.id.hex()} Kind={item.kind} Orientation={item.orientation} Position={item.position} Alignment={item.alignment} Extents={item.extents}')
    print('Location')
    print(item.location)
    print(f'Meshes: {len(item.meshes)}')
    print(f'Collider meshes: {len(item.collider_meshes)}')

    for mesh in item.meshes:
        mesh.unpack()
        hl2ss_3dcv.su_normalize(mesh, item.location @ result.pose)
        open3d_mesh = hl2ss_sa.su_mesh_to_open3d_triangle_mesh(mesh)
        open3d_mesh = hl2ss_sa.open3d_triangle_mesh_swap_winding(open3d_mesh)
        open3d_mesh.compute_vertex_normals()
        open3d_mesh.paint_uniform_color(kind_color[int(item.kind)])
        open3d_meshes.append(open3d_mesh)

o3d.visualization.draw_geometries(open3d_meshes, mesh_show_back_face=False)

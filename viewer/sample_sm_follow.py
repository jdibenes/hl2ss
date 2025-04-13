#------------------------------------------------------------------------------
# This script demonstrates how to continuously acquire Spatial Mapping data in
# the users vicinity.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import open3d as o3d
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_sa
import hl2ss_utilities

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Spatial Mapping manager parameters
tpcm = 500
radius = 1

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------   
    listener = hl2ss_utilities.key_listener(keyboard.Key.space)
    listener.open()
    
    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    first_geometry = True

    # Start interfaces --------------------------------------------------------
    sm_manager = hl2ss_sa.sm_manager_mp(host, hl2ss.IPCPort.SPATIAL_MAPPING, triangles_per_cubic_meter=tpcm)
    sm_manager.open()

    sink_si = hl2ss_mp.stream(hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))
    sink_si.open()

    # Main loop ---------------------------------------------------------------
    while (not listener.pressed()):
        vis.poll_events()
        vis.update_renderer()

        # Get Spatial Input data ----------------------------------------------
        fs_si, data_si = sink_si.get_most_recent_frame()
        if (data_si is None):
            continue

        # Set new volume ------------------------------------------------------
        si = data_si.payload
        origin = si.head_pose.position

        # Get surfaces in new volume ------------------------------------------
        volume = hl2ss.sm_bounding_volume()
        volume.add_sphere(origin, radius) # Sample 3D sphere centered on head
        sm_manager.set_volumes(volume)
        sm_manager.get_observed_surfaces()
        if (not sm_manager.get_updated_flag()):
            continue

        # Update visualization ------------------------------------------------
        vis.clear_geometries()

        meshes = sm_manager.get_meshes()
        meshes = [hl2ss_sa.open3d_triangle_mesh_swap_winding(hl2ss_sa.sm_mesh_to_open3d_triangle_mesh(mesh)) for mesh in meshes]
        for mesh in meshes:
            mesh.vertex_colors = mesh.vertex_normals
            vis.add_geometry(mesh, first_geometry)

        if (len(meshes) > 0):
            first_geometry = False

    # Stop streams ------------------------------------------------------------
    sink_si.close()
    sm_manager.close()

    # Stop keyboard events ----------------------------------------------------
    listener.close()

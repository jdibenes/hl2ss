#------------------------------------------------------------------------------
# This script demonstrates how to continuously acquire Spatial Mapping data in
# the users vicinity.
# Press space to stop.
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import open3d as o3d
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_sa

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Buffer length in seconds
buffer_size = 10

# Spatial Mapping manager parameters
tpcm = 500
threads = 2
radius = 0.5

#------------------------------------------------------------------------------

if __name__ == '__main__':
    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    # Create Open3D visualizer ------------------------------------------------
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().mesh_show_back_face = True

    first_geometry = True

    # Start interfaces --------------------------------------------------------
    sm_manager = hl2ss_sa.sm_manager(host, tpcm, threads)
    sm_manager.open()

    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.SPATIAL_INPUT, hl2ss_lnm.rx_si(host, hl2ss.StreamPort.SPATIAL_INPUT))
    producer.initialize(hl2ss.StreamPort.SPATIAL_INPUT, buffer_size * hl2ss.Parameters_SI.SAMPLE_RATE)
    producer.start(hl2ss.StreamPort.SPATIAL_INPUT)

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sink_si = consumer.create_sink(producer, hl2ss.StreamPort.SPATIAL_INPUT, manager, ...)
    sink_si.get_attach_response()

    # Main loop ---------------------------------------------------------------
    while (enable):
        # Get Spatial Input data ----------------------------------------------
        sink_si.acquire()

        _, data_si = sink_si.get_most_recent_frame()
        if (data_si is None):
            continue

        # Set new volume ------------------------------------------------------
        si = hl2ss.unpack_si(data_si.payload)
        origin = si.get_head_pose().position

        volume = hl2ss.sm_bounding_volume()
        volume.add_sphere(origin, radius) # Sample 3D sphere centered on head

        sm_manager.set_volumes(volume)
        
        # Get surfaces in new volume ------------------------------------------
        sm_manager.get_observed_surfaces()

        # Update visualization ------------------------------------------------
        vis.clear_geometries()

        meshes = sm_manager.get_meshes()
        meshes = [hl2ss_sa.sm_mesh_to_open3d_triangle_mesh(mesh) for mesh in meshes]
        for mesh in meshes:
            mesh.vertex_colors = mesh.vertex_normals
            vis.add_geometry(mesh, first_geometry)

        if (len(meshes) > 0):
            first_geometry = False

        vis.poll_events()
        vis.update_renderer()

    # Stop streams ------------------------------------------------------------
    sink_si.detach()
    producer.stop(hl2ss.StreamPort.SPATIAL_INPUT)

    # Stop keyboard events ----------------------------------------------------
    listener.join()

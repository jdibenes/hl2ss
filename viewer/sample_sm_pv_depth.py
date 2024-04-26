#------------------------------------------------------------------------------
# This script demonstrates how to compute a depth map for a given PV image
# using the Spatial Mapping data from the HoloLens. Note that depth map 
# generation will fail outside the SM sampling volume.
#------------------------------------------------------------------------------

import multiprocessing as mp
import numpy as np
import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv
import hl2ss_sa
import hl2ss_utilities


# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.7'

# Calibration folder (must exist but can be empty)
calibration_path = '../calibration'

# PV settings
pv_focus = 1000 # In mm
pv_width = 640
pv_height = 360
pv_fps = 30

# Buffer length in seconds
buffer_size = 10 

# SM settings
sm_tpcm = 1000 # Triangles per cubic meter
sm_threads = 2 # Number of threads for computing meshes (on the HoloLens)
sm_origin = [0, 0, 0] # Origin of sampling volume
sm_radius = 5 # Radius of sampling volume 

#------------------------------------------------------------------------------

if __name__ == "__main__":
    # Create and configure SM manager -----------------------------------------
    sm_manager = hl2ss_sa.sm_mp_manager(host, sm_tpcm, sm_threads)
    sm_manager.open()

    # Sampling volume
    sm_volumes = hl2ss.sm_bounding_volume()
    sm_volumes.add_sphere(sm_origin, sm_radius)
    sm_manager.set_volumes(sm_volumes)
    sm_manager.get_observed_surfaces()

    # Start PV stream ---------------------------------------------------------
    # Start PV subsystem
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Fix PV focus
    rc_client = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    rc_client.open()
    rc_client.wait_for_pv_subsystem(True)
    rc_client.set_pv_focus(hl2ss.PV_FocusMode.Manual, hl2ss.PV_AutoFocusRange.Normal, hl2ss.PV_ManualFocusDistance.Infinity, pv_focus, hl2ss.PV_DriverFallback.Disable)
    rc_client.close()

    # Get calibration (focus is fixed so intrinsics don't change between frames)
    pv_calibration = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, calibration_path, pv_focus, pv_width, pv_height, pv_fps)
    pv_calibration.intrinsics, pv_calibration.extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_calibration.intrinsics, np.eye(4, 4, dtype=np.float32))

    # Get PV rays in camera coordinates
    pv_uv2xy = hl2ss_3dcv.compute_uv2xy(pv_calibration.intrinsics, pv_width, pv_height)
    pv_xy1 = hl2ss_3dcv.to_homogeneous(pv_uv2xy)
    pv_rays = hl2ss_3dcv.to_unit(pv_xy1)

    # Create windows
    wnd_name_pv = 'PV'
    wnd_name_depth = 'PV-SM Depth'

    cv2.namedWindow(wnd_name_pv)
    cv2.namedWindow(wnd_name_depth)

    pc = hl2ss_utilities.framerate_counter()

    # Start PV capture
    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_fps))
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, buffer_size * pv_fps)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)

    consumer = hl2ss_mp.consumer()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, mp.Manager(), None)
    sink_pv.get_attach_response()
    
    pc.reset()

    # Acquire data ------------------------------------------------------------
    while (True):
        key = cv2.waitKey(1)
        if ((key & 0xff) == 27): # Esc to stop
            break

        _, data = sink_pv.get_most_recent_frame()
        
        if ((data is None) or (not hl2ss.is_valid_pose(data.pose))):
            pc.reset()
            continue

        camera2world = hl2ss_3dcv.camera_to_rignode(pv_calibration.extrinsics) @ hl2ss_3dcv.reference_to_world(data.pose)

        # Compute PV camera rays in world coordinates
        pv_world_origins = np.tile(camera2world[3, :3], (pv_height, pv_width, 1))
        pv_world_directions = pv_rays @ camera2world[:3, :3]
        pv_world_rays = np.dstack((pv_world_origins, pv_world_directions))

        # Update SM surfaces
        sm_manager.get_observed_surfaces()

        # Obtain PV depth via raycasting
        pv_depth = sm_manager.cast_rays(pv_world_rays)
        pv_depth[np.isinf(pv_depth)] = 0

        # Display images
        cv2.imshow(wnd_name_pv, data.payload.image)
        cv2.imshow(wnd_name_depth, pv_depth / np.max(pv_depth)) # Normalized for visibility

        pc.increment()
        if (pc.delta() > 5.0):
            print(f'fps: {pc.get()}')
            pc.reset()
        
    # Stop PV stream ----------------------------------------------------------
    sink_pv.detach()
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Close SM manager --------------------------------------------------------
    sm_manager.close()

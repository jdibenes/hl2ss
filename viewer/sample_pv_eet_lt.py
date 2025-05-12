#------------------------------------------------------------------------------
# Extended eye tracking projection onto PV images using Long Throw depth for
# raycasting.
# Press esc to stop.
#------------------------------------------------------------------------------

import numpy as np
import open3d as o3d
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_mp
import hl2ss_utilities

# Settings --------------------------------------------------------------------

host = '192.168.1.7'

calibration_path = '../calibration'

pv_width     = 760
pv_height    = 428
pv_framerate = 30

eet_fps = 90

radius         = 5
combined_color = (255, 0, 255)
thickness      = -1

#------------------------------------------------------------------------------

if __name__ == '__main__':
    hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    calibration_lt = hl2ss_3dcv.get_calibration_rm(calibration_path, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    uv2xy = calibration_lt.uv2xy
    xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration_lt.scale)

    sink_pv  = hl2ss_mp.stream(hl2ss_lnm.rx_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, width=pv_width, height=pv_height, framerate=pv_framerate, decoded_format='bgr24'))
    sink_lt  = hl2ss_mp.stream(hl2ss_lnm.rx_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW))
    sink_eet = hl2ss_mp.stream(hl2ss_lnm.rx_eet(host, hl2ss.StreamPort.EXTENDED_EYE_TRACKER, fps=eet_fps))

    sink_pv.open()
    sink_lt.open()
    sink_eet.open()

    pv_intrinsics = hl2ss_3dcv.pv_create_intrinsics_placeholder()
    pv_extrinsics = np.eye(4, 4, dtype=np.float32)

    last_lt_ts = 0

    cv2.namedWindow('Video')

    vi_counter = hl2ss_utilities.framerate_counter()
    vi_counter.reset()

    while ((cv2.waitKey(1) & 0xFF) != 27):
        _, data_pv = sink_pv.get_most_recent_frame()
        color = None
        world_to_pv_image = None
        if ((data_pv is not None)):
            color = data_pv.payload.image
            if (hl2ss.is_valid_pose(data_pv.pose)):
                pv_intrinsics                      = hl2ss_3dcv.pv_update_intrinsics(pv_intrinsics, data_pv.payload.focal_length, data_pv.payload.principal_point)
                color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
                world_to_pv_image = hl2ss_3dcv.world_to_reference(data_pv.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)

        _, data_eet = sink_eet.get_most_recent_frame()
        eet = None
        eet_pose = None
        if ((data_eet is not None) and (hl2ss.is_valid_pose(data_eet.pose))):
            eet = data_eet.payload
            eet_pose = data_eet.pose

        _, data_lt = sink_lt.get_most_recent_frame()
        rcs = None
        if ((data_lt is not None) and (hl2ss.is_valid_pose(data_lt.pose)) and (data_lt.timestamp != last_lt_ts)):
            last_lt_ts = data_lt.timestamp
            
            depth = data_lt.payload.depth

            lt_to_world  = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose)
            points       = hl2ss_3dcv.rm_depth_to_points(xy1, hl2ss_3dcv.rm_depth_normalize(depth, scale))
            world_points = hl2ss_3dcv.transform(points, lt_to_world)
            
            h, w  = depth.shape[-2:]
            mask  = depth > 0
            faces = []

            for i in range(1, h):
                for j in range(1, w):
                    ul,    ur,    bl,    br    = (i-1, j-1), (i-1, j), (i, j-1), (i, j)
                    ul_i,  ur_i,  bl_i,  br_i  = ul[0] * w + ul[1], ur[0] * w + ur[1], bl[0] * w + bl[1], br[0] * w + br[1]
                    ul_cc, ur_cc, bl_cc, br_cc = mask[ul[0], ul[1]], mask[ur[0], ur[1]], mask[bl[0], bl[1]], mask[br[0], br[1]]
                    
                    #d1 = hl2ss_3dcv.compute_norm(world_points[ul[0], ul[1], :] - world_points[br[0], br[1], :]) if (ul_cc and br_cc) else 0
                    #d2 = hl2ss_3dcv.compute_norm(world_points[ur[0], ur[1], :] - world_points[bl[0], bl[1], :]) if (ur_cc and bl_cc) else 0

                    v1_i, v2_i, v1_cc, v2_cc = (ur_i, bl_i, ur_cc, bl_cc) #if (d2 <= d1) else (ul_i, br_i, ul_cc, br_cc)

                    if (bl_cc and br_cc and v1_cc):
                        faces.append([bl_i, br_i, v1_i])
                    if (ur_cc and ul_cc and v2_cc):
                        faces.append([ur_i, ul_i, v2_i])

            open3d_mesh           = o3d.geometry.TriangleMesh()
            open3d_mesh.vertices  = o3d.utility.Vector3dVector(hl2ss_3dcv.block_to_list(world_points))
            open3d_mesh.triangles = o3d.utility.Vector3iVector(np.array(faces, dtype=np.uint32))
            
            rcs = o3d.t.geometry.RaycastingScene()
            rcs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(open3d_mesh))

        if (color is not None):
            if ((world_to_pv_image is not None) and (eet is not None) and (eet_pose is not None) and (rcs is not None)):
                if (eet.combined_ray_valid):
                    local_combined_ray = hl2ss_3dcv.si_ray_to_vector(eet.combined_ray.origin, eet.combined_ray.direction)
                    combined_ray = hl2ss_3dcv.si_ray_transform(local_combined_ray, eet_pose)
                    d = rcs.cast_rays(combined_ray)['t_hit'].numpy()
                    if (np.isfinite(d)):
                        combined_point = hl2ss_3dcv.si_ray_to_point(combined_ray, d)
                        combined_image_point = hl2ss_3dcv.project(combined_point, world_to_pv_image)
                        hl2ss_utilities.draw_points(color, combined_image_point.astype(np.int32), radius, combined_color, thickness)

            cv2.imshow('Video', color)

        vi_counter.increment()
        if (vi_counter.delta() > 2.0):
            print(f'FPS: {vi_counter.get()}')
            vi_counter.reset()
            
    sink_pv.close()
    sink_lt.close()
    sink_eet.close()

    hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

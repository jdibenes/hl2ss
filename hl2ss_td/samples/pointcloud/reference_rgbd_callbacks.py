import numpy as np
import cv2
import hl2ss
import hl2ss_3dcv

def onData(comp, data):
	core_lt = op('hl2ss_depth_to_points/hl2ss_core_lt').ext.Hl2ssExt
	core_pv = op('hl2ss_core_pv1').ext.Hl2ssExt

	# PV intrinsics and extrinsics ---------------------------------
	pv_intrinsics = hl2ss.create_pv_intrinsics_placeholder()
	pv_extrinsics = np.eye(4, 4, dtype=np.float32)
	# PV intrinsics may change between frames due to autofocus
	pv_intrinsics = hl2ss.update_pv_intrinsics(pv_intrinsics, core_pv.Data.payload.focal_length, core_pv.Data.payload.principal_point)
	color_intrinsics, color_extrinsics = hl2ss_3dcv.pv_fix_calibration(pv_intrinsics, pv_extrinsics)
	color = core_pv.Data.payload.image

	uv2xy = core_lt.Calibration.uv2xy
	# uv2xy = hl2ss_3dcv.compute_uv2xy(core_lt.Calibration.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
	
	# xy1 adds 3rd dimension set to 1
	# scale is defined as lengths of xy1 * calibration.scale
	xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, core_lt.Calibration.scale)

	depth = core_lt.Data.payload.depth
	# depth = hl2ss_3dcv.rm_depth_undistort(core_lt.Data.payload.depth, core_lt.Calibration.undistort_map)

	# normalized depth = absolute depth / scale
	norm_depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)

	# just rays * norm_depth
	lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, norm_depth)
	
	lt_to_world       = hl2ss_3dcv.camera_to_rignode(core_lt.Calibration.extrinsics) @ hl2ss_3dcv.reference_to_world(core_lt.Data.pose)
	world_to_pv_image = hl2ss_3dcv.world_to_reference(core_pv.Data.pose) @ hl2ss_3dcv.rignode_to_camera(color_extrinsics) @ hl2ss_3dcv.camera_to_image(color_intrinsics)
	world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
	pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
	color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR)

	core_lt.processImage(world_points, op('script_reference_points'))
	core_lt.processImage(color, op('script_reference_colors'))

def onCalibration(comp, calibration):
	return
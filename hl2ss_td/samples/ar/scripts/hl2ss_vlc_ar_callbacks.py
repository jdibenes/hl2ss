import numpy as np
import cv2
import hl2ss_3dcv

def onData(comp, data):
	# calculate and save sensor pose to CHOP
	mtx = hl2ss_3dcv.camera_to_rignode(comp.Calibration.extrinsics) @ data.pose
	# Hl2ss uses row-major format, but TD uses column-major. Matrix2Chop
	# basically transposes matrix as it reads numpy matrix row-by-row and writes
	# to CHOP column-by-column.
	matrix2Chop(mtx.ravel(), op('constant_pose'))

def onCalibration(comp, calibration):
	mtx = calibration.intrinsics[:3, :3].transpose()
	width = 640
	height = 480
	near = 0.1
	far = 1000
	projMtx = camIntrinsics2ProjMatrix(mtx, width, height, near, far)
	matrix2Chop(projMtx.vals, op('constant_proj_mtx'))


def matrix2Chop(mtx, constChop):
	constChop.seq.const.numBlocks = 16
	mtxNames = ['m00', 'm10', 'm20', 'm30', 'm01', 'm11', 'm21', 'm31', 'm02', 'm12', 'm22', 'm32', 'm03', 'm13', 'm23', 'm33']
	for i in range(16):
		constChop.seq.const[i].par.name = mtxNames[i]
		constChop.seq.const[i].par.value = mtx[i]
			
def camIntrinsics2ProjMatrix(mtx, width, height, near, far) -> tdu.Matrix:
	"""Converts camera intrinsics to projection matrix.
	Conversion is copied from kinect calibration.
	More info: https://forum.derivative.ca/t/projection-matrix-dilemma/4243/9

	Args:
		mtx: OpenCV camera intrinsic matrix
		width: Image width in pixels
		height: Image height in pixels
		near: Near clipping plane
		far: Far clipping plane

	Returns:
		Projection matrix
	"""
	fovx, fovy, focalLength, principalPoint, aspectRatio = cv2.calibrationMatrixValues(mtx, (width, height), width, height)

	matrix = tdu.Matrix()
	if fovx != 0.0:
		fovx = focalLength
		fovy = fovx
		cx = principalPoint[0]
		cy = principalPoint[1]
	
		n = near
		f = far
		w = width
		h = height
	
		l = n * (-cx) / fovx
		r = n * (w - cx) / fovx
		b = n * (cy - h) / fovy
		t = n * (cy) / fovy
	
		A = (r + l) / (r - l)
		B = (t + b) / (t - b)
		C = (f + n) / (n - f)
		D = (2*f*n)/(n - f)
	
		nrl = (2*n)/(r-l)
		ntb = (2*n)/(t-b)
			
		matrix = tdu.Matrix([nrl,0,0,0],[0,ntb,0,0],[A,B,C,-1],[0,0,D,0])
	return matrix
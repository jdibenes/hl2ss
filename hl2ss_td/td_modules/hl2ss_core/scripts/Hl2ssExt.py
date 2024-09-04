# add paths for external python libraries
extLibs = [f'{parent().fileFolder}/../../../viewer', f'{parent().fileFolder}/../../venv/Lib/site-packages']
for extLib in extLibs:
	if extLib not in sys.path:
		sys.path = extLibs + sys.path

import numpy as np
import select
from collections import deque
import hl2ss
import hl2ss_lnm
import hl2ss_3dcv
import hl2ss_mp
import hl2ss_utilities
import json

class Hl2ssExt:
	"""
	Wrapper around hl2ss python library - allowing its usage within TD.
	"""
	def __init__(self, ownerComp):
		self.ownerComp = ownerComp
		self.data = None
		self.calibration = None
		
		self.client = None
		self.buffer = deque(maxlen = self.ownerComp.par.Bufferlen.eval())
		self.framestamp = 0
		self.lastPresentedFramestamp = 0
		self._activeSubsystemPv = False

		# multiprocessing
		self.producer = hl2ss_mp.producer()
		self.sink = None

		# audio
		# used for re-framing audio samples
		self.prevReadSamples = 0

		# initial cleanup
		op('metadata').text = op('default_metadata').text
		op('calibration').text = op('default_calibration').text
		op('script_chop').lock = True
		op('script_chop').clear()
		op('script_chop').lock = False
		self.ownerComp.par.Stream = False

	def __delTD__(self):
		"""Triggered before extension is destroyed. Perform cleanup.
		"""
		self.CloseStream()

	@property
	def Data(self):
		"""Access original hl2ss data.
		"""
		return self.data
	
	@property
	def Calibration(self):
		"""Access original hl2ss calibration.
		"""
		return self.calibration
	
	@property
	def port(self):
		return int(self.ownerComp.par.Port.eval())
	
	@property
	def threadMode(self):
		return self.ownerComp.par.Threadmode.eval()
	
	@property
	def activeSubsystemPv(self):
		return self._activeSubsystemPv
	
	@activeSubsystemPv.setter
	def activeSubsystemPv(self, value: bool):
		if self.port != hl2ss.StreamPort.PERSONAL_VIDEO:
			raise ValueError('Wrong port selected. To start / stop subsystem pv, select stream port PERSONAL_VIDEO.')
		if value != self._activeSubsystemPv:
			if value:
				hl2ss_lnm.start_subsystem_pv(
					self.ownerComp.par.Address.eval(),
					self.port,
					enable_mrc=self.ownerComp.par.Enablemrc.eval(),
					shared=self.ownerComp.par.Shared.eval()
				)
			else:
				hl2ss_lnm.stop_subsystem_pv(
					self.ownerComp.par.Address.eval(),
					self.port
				)
			self._activeSubsystemPv = value
	
	def getPVFormat(self):
		format = self.ownerComp.par.Format.eval()
		resolution = format.split('@')[0].split('x')
		width = int(resolution[0])
		height = int(resolution[1])
		framerate = int(format.split('@')[1])
		return width, height, framerate
	
	def getClient(self):
		# HoloLens address
		host = self.ownerComp.par.Address.eval()

		# Operating mode
		# 0: video
		# 1: video + rig pose
		# 2: query calibration (single transfer)
		mode = hl2ss.StreamMode.MODE_1

		# Framerate denominator (must be > 0)
		# Effective framerate is framerate / divisor
		divisor = 1

		# Video encoding profile
		profile = int( self.ownerComp.par.Videoprofile.eval() )

		# PV | camera parameters
		# Ignored in shared mode
		width, height, framerate = self.getPVFormat()

		# PV | Decoded format
		decodedFormat = self.ownerComp.par.Colorformat.eval()

		# audio settings
		audioProfile = int( self.ownerComp.par.Audioprofile.eval() )
		aacLevel = int( self.ownerComp.par.Aaclevel.eval() )

		if self.port in (
			hl2ss.StreamPort.RM_VLC_LEFTFRONT,
			hl2ss.StreamPort.RM_VLC_LEFTLEFT,
			hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
			hl2ss.StreamPort.RM_VLC_RIGHTRIGHT
			):
			client = hl2ss_lnm.rx_rm_vlc(host, self.port, mode=mode, divisor=divisor, profile=profile)
		
		elif self.port == hl2ss.StreamPort.RM_DEPTH_AHAT:
			client = hl2ss_lnm.rx_rm_depth_ahat(host, self.port, mode=mode, divisor=divisor, profile_z=hl2ss.DepthProfile.SAME, profile_ab=profile)
		
		elif self.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW:
			client = hl2ss_lnm.rx_rm_depth_longthrow(host, self.port, mode=mode, divisor=divisor)

		elif self.port in (
			hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
			hl2ss.StreamPort.RM_IMU_GYROSCOPE,
			hl2ss.StreamPort.RM_IMU_MAGNETOMETER
			):
			client = hl2ss_lnm.rx_rm_imu(host, self.port, mode=mode)
			
		elif self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			client = hl2ss_lnm.rx_pv(host, self.port, mode=mode, width=width, height=height, framerate=framerate, divisor=divisor, profile=profile, decoded_format=decodedFormat)
		
		elif self.port == hl2ss.StreamPort.MICROPHONE:
			client = hl2ss_lnm.rx_microphone(host, self.port, profile=audioProfile, level=aacLevel)

		return client
	
	def GetCalibration(self):
		host = self.ownerComp.par.Address.eval()
		calibFolder = self.ownerComp.par.Calibfolder.evalFile().path

		if self.port == hl2ss.StreamPort.MICROPHONE:
			# has no calibration data
			return

		# store calibration for any downstream python-based cv
		if self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			width, height, framerate = self.getPVFormat()
			prevActiveSubsystemPv = self.activeSubsystemPv
			self.activeSubsystemPv = True

			try:
				# focus is set to 0 as it doesn't seem to be used in hl2ss_3dcv
				self.calibration = hl2ss_3dcv.get_calibration_pv(host, self.port, calibFolder, 0, width, height, framerate)
				self.ownerComp.clearScriptErrors(recurse=False, error="Getting PV calibration failed.")
			except:
				# calibration wasn't found on disk and downloading from HL failed
				self.activeSubsystemPv = False # cleanup subsystem PV
				self.ownerComp.addScriptError("Getting PV calibration failed.")
			
			self.activeSubsystemPv = prevActiveSubsystemPv
		else:
			try:
				self.calibration = hl2ss_3dcv.get_calibration_rm(host, self.port, calibFolder)
				self.ownerComp.clearScriptErrors(recurse=False, error="Getting RM calibration failed.")
			except:
				# calibration wasn't found on disk and downloading from HL failed
				self.ownerComp.addScriptError("Getting RM calibration failed.")

		self.handleCalibration()

		callbacks = self.ownerComp.par.Callbacks.eval()
		if callbacks is not None:
			mod(callbacks.path).onCalibration(self.ownerComp, self.calibration)

	def OpenStream(self):
		if (self.threadMode == 'sync' and self.client is not None) or (self.threadMode == 'mp' and self.sink is not None):
			# stream is already open
			return

		if self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			self.activeSubsystemPv = True # start subsystem PV
		if self.ownerComp.par.Autogetcalib:
			self.GetCalibration()
		self.framestamp = 0
		self.lastPresentedFramestamp = 0

		match self.threadMode:
			case 'sync':
				self.buffer = deque(maxlen = self.ownerComp.par.Bufferlen.eval())
				self.prevReadSamples = 0
				self.client = self.getClient()
				try:
					self.client.open()
					self.ownerComp.color = (0.05, 0.5, 0.2)
					self.ownerComp.clearScriptErrors(recurse=False, error="Opening of stream failed.")
				except:
					self.client = None
					if self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
						self.activeSubsystemPv = False # cleanup subsystem PV
					self.ownerComp.par.Stream = False
					self.ownerComp.addScriptError("Opening of stream failed.")
			
			case 'mp':
				client = self.getClient()
				manager = self.ownerComp.par.Mpmanager.eval().Manager
				consumer = hl2ss_mp.consumer()
				self.producer.configure( self.port, client )
				self.producer.initialize( self.port, self.ownerComp.par.Bufferlen.eval() )
				self.producer.start( self.port )
				self.sink = consumer.create_sink(self.producer, self.port, manager, None)
				self.sink.get_attach_response()
				while (self.sink.get_buffered_frame(0)[0] != 0):
					pass
				self.ownerComp.color = (0.35, 0.5, 0.05)

	def CloseStream(self):
		self.ownerComp.par.Stream = False
		self.ownerComp.color = (0.55, 0.55, 0.55)

		if (self.threadMode == 'sync' and self.client is None) or (self.threadMode == 'mp' and self.sink is None):
			# stream is already closed
			return

		match self.threadMode:
			case 'sync':
				self.client.close()
				self.client = None
			
			case 'mp':
				self.sink.detach()
				self.sink = None
				self.producer.stop(self.port)

		if self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			# cleanup subsystem PV
			self.activeSubsystemPv = False

	def OnTdFrame(self) -> bool:
		"""Try to receive new data and process next frame.
		This method needs to run on each TD frame.

		Returns:
		    bool: Success of providing new data.
		"""
		if self.threadMode == 'sync' and self.client is not None:
			self.recvData(self.client)

		if self.port == hl2ss.StreamPort.MICROPHONE:
			# Skips the frame picker pipeline in order to reliably read (and
			# re-frame) all audio samples from queue.
			return self.GetAndPresentAudio()
		else:
			return self.GetAndPresentFrame(extObj=self)
			
	def recvData(self, client):
		"""Try to receive data from client using "semi-non-blocking" approach.
		Semi-non-blocking means it won't wait (and block) if first chunk of next
		packet isn't available. However if first chunk is available, it will
		block until full packet (all chunks) is received. Since chunks usually
		arrive closely thogether, this kinda works without slowing TD down.
		"""
		while self.socketSelect(client):
			data = client.get_next_packet()
			self.dataPreprocessor(data)
			self.buffer.append(data)
			self.framestamp += 1

	def dataPreprocessor(self, data):
		"""Pre-process data before adding them into buffer.
		"""
		if self.port == hl2ss.StreamPort.MICROPHONE and len(data.payload) > 1:
			# convert planar channels (in case of AAC) into a unified packed data
			data.payload = hl2ss_utilities.microphone_planar_to_packed(data.payload)
		return data
	
	def socketSelect(self, client) -> bool:
		"""Check if client's socket has some data available (non-blocking).
		"""
		# get underling socket of client
		# rx_rm_vlc._client -> _gatherer._client -> _client._socket
		socket = client._client._client._socket
		
		# instead of blocking with recv, check if sockets has available data
		# toRead will be a list of sockets with readable data
		toRead, toWrite, errors = select.select([socket], [], [], 0)
		return len(toRead) > 0
	
	def GetAndPresentFrame(self, extObj) -> bool:
		"""Pick next frame (if available) and store it for downstream
		processing.

		Args:
		    extObj: Specifies which extension object will be used for frame
		    picker settings, final data storage and callback trigger. Its
		    purpose is to enable decoupled frame picker fuctionality.

		Returns:
		    bool: Success of providing new data.
		"""
		if self.threadMode == 'sync' and (self.client is None or len(self.buffer) == 0):
			return False
		elif self.threadMode == 'mp' and self.sink is None:
			return False

		pickerMode = extObj.ownerComp.par.Framepicker.eval()
		refMetadata = extObj.ownerComp.op('json_ref_metadata')
		fs, data = self.getFrame(pickerMode, refMetadata, extObj.ownerComp)

		if data is not None and fs > extObj.lastPresentedFramestamp:
			# store data for any downstream python-based cv
			extObj.data = data
			# present data to TD
			self.handleData(fs, data, targetComp=extObj.ownerComp)
			extObj.lastPresentedFramestamp = fs

			callbacks = extObj.ownerComp.par.Callbacks.eval()
			if callbacks is not None:
				mod(callbacks.path).onData(extObj.ownerComp, data)

			return True
		return False
	
	def getFrame(self, pickerMode: str, refMetadata: DAT, ownerComp):
		match pickerMode:
			case 'getLatestFrame':
				return self.getLatestFrame()
			case 'getNearestFrame':
				targetTimestamp = int(refMetadata.source['timestamp'])
				return self.getNearestFrame(targetTimestamp, ownerComp)
			case 'getBufferedFrame':
				targetFramestamp = int(refMetadata.source['framestamp'])
				return self.getBufferedFrame(targetFramestamp)
			case _:
				return (None, None)

	def getLatestFrame(self):
		match self.threadMode:
			case 'sync':
				return (self.framestamp, self.buffer[-1])
			case 'mp':
				return self.sink.get_most_recent_frame()
	
	def getNearestFrame(self, timestamp, ownerComp):
		timePreference = ownerComp.par.Timepreference.menuIndex
		tiebreakRight = ownerComp.par.Tiebreakright.eval()
		match self.threadMode:
			case 'sync':
				index = hl2ss_mp._get_nearest_packet(self.buffer, timestamp, timePreference, tiebreakRight)
				return (None, None) if (index is None) else (self.framestamp - len(self.buffer) + 1 + index, self.buffer[index])
			case 'mp':
				return self.sink.get_nearest(timestamp, timePreference, tiebreakRight)
	
	def getBufferedFrame(self, framestamp):
		match self.threadMode:
			case 'sync':
				n = len(self.buffer)
				index = n - 1 - self.framestamp + framestamp
				return (-1, None) if (index < 0) else (1, None) if (index >= n) else (framestamp, self.buffer[index])
			case 'mp':
				state, data = self.sink.get_buffered_frame(framestamp)
				return (framestamp, data) if state == 0 else (state, data)
	
	def handleData(self, framestamp: int, data: hl2ss._packet, targetComp: COMP = None):
		"""Push hl2ss data to TD.
		"""
		if targetComp == None:
			targetComp = self.ownerComp

		if self.port in (
			hl2ss.StreamPort.RM_VLC_LEFTFRONT,
			hl2ss.StreamPort.RM_VLC_LEFTLEFT,
			hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
			hl2ss.StreamPort.RM_VLC_RIGHTRIGHT
			):
			self.processImage(data.payload.image, targetComp.op('script_img1'))
			metadata = {
				'framestamp': framestamp,
				'timestamp': data.timestamp,
				'pose': data.pose.tolist(),
				'sensor_ticks': data.payload.sensor_ticks.tolist(),
				'exposure': data.payload.exposure.tolist(),
				'gain': data.payload.gain.tolist()
			}

		elif self.port == hl2ss.StreamPort.RM_DEPTH_AHAT or self.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW:
			self.processImage(data.payload.depth, targetComp.op('script_img1'))
			self.processImage(data.payload.ab, targetComp.op('script_img2'))
			metadata = {
				'framestamp': framestamp,
				'timestamp': data.timestamp,
				'pose': data.pose.tolist(),
				'sensor_ticks': data.payload.sensor_ticks.tolist()
			}

		elif self.port in (
			hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
			hl2ss.StreamPort.RM_IMU_GYROSCOPE,
			hl2ss.StreamPort.RM_IMU_MAGNETOMETER
			):
			self.processImu(data.payload, targetComp.op('script_chop'))
			metadata = {
				'framestamp': framestamp,
				'timestamp': data.timestamp,
				'pose': data.pose.tolist()
			}

		elif self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			self.processImage(data.payload.image, targetComp.op('script_img1'))
			metadata = {
				'framestamp': framestamp,
				'timestamp': data.timestamp,
				'pose': data.pose.tolist(),
				'focal_length': data.payload.focal_length.tolist(),
				'principal_point': data.payload.principal_point.tolist(),
				'exposure_time': data.payload.exposure_time.tolist(),
				'exposure_compensation': data.payload.exposure_compensation.tolist(),
				'lens_position': data.payload.lens_position.tolist(),
				'focus_state': data.payload.focus_state.tolist(),
				'iso_speed': data.payload.iso_speed.tolist(),
				'white_balance': data.payload.white_balance.tolist(),
				'iso_gains': data.payload.iso_gains.tolist(),
				'white_balance_gains': data.payload.white_balance_gains.tolist()
			}

		targetComp.op('metadata').text = json.dumps(metadata, indent=4)

	def processImage(self, img, scriptOp):
		"""Push image into Script TOP.
		"""
		if img.ndim == 2:
			# contains mono image, needs 3rd dimension (single channel) for each value
			img = np.expand_dims(img, 2)
		scriptOp.lock = True
		scriptOp.copyNumpyArray(img)
		scriptOp.lock = False

	def processImu(self, payload, scriptOp):
		"""Push IMU data into Script CHOP.
		vinyl_hup_ticks and soc_ticks are skipped (won't fit into floats).
		"""
		imuData = hl2ss.unpack_rm_imu(payload)
		count = imuData.get_count()
		chopData = np.zeros((4, count), dtype=np.float32)
		for i in range(count):
			sample = imuData.get_frame(i)
			chopData[0, i] = sample.x
			chopData[1, i] = sample.y
			chopData[2, i] = sample.z
			chopData[3, i] = sample.temperature
		scriptOp.lock = True
		scriptOp.copyNumpyArray(chopData, baseName='chan')
		scriptOp.lock = False

	def GetAndPresentAudio(self):
		"""Read & re-frame audio samples from buffer, then push them into CHOPs.
		Re-framing is required in order to match TD frame rate.

		Returns:
		    bool: Success of providing new data.
		"""
		if self.threadMode == 'sync' and (self.client is None or len(self.buffer) == 0):
			return False
		elif self.threadMode == 'mp' and self.sink is None:
			raise ValueError("Multiprocessing isn't currently supported by audio pipeline.")
		
		audioProfile = int( self.ownerComp.par.Audioprofile.eval() )
		aacLevel = int( self.ownerComp.par.Aaclevel.eval() )
		targetCount = int( hl2ss.Parameters_MICROPHONE.SAMPLE_RATE / project.cookRate )
		if audioProfile == hl2ss.AudioProfile.RAW:
			channelCount = hl2ss.Parameters_MICROPHONE.ARRAY_CHANNELS if aacLevel == hl2ss.AACLevel.L5 else hl2ss.Parameters_MICROPHONE.CHANNELS
		else:
			channelCount = hl2ss.Parameters_MICROPHONE.CHANNELS

		audioSamples = self.readAudioSamples(targetCount, channelCount)
		if len(audioSamples) > 0:
			# store data for any downstream python-based processing
			# since original data likely won't be useful within TD workflows, store re-framed samples
			self.data = audioSamples
			# present data to TD
			self.processAudio(audioSamples, channelCount, self.ownerComp.op('script_chop'))

			callbacks = self.ownerComp.par.Callbacks.eval()
			if callbacks is not None:
				mod(callbacks.path).onData(self.ownerComp, audioSamples)
			return True
		return False
	
	def readAudioSamples(self, targetCount, channelCount) -> np.array:
		"""Read specified number of audio samples from buffer.
		If buffer doesn't have enough samples, empty array is returned.

		Args:
		    targetCount: Number of samples to read.
			channelCount: Number of channels within a buffer.

		Returns:
		    Requested audio samples.
		"""
		samples = np.array([], self.buffer[-1].payload.dtype)
		
		if self.getAvailableAudioSamplesCount(channelCount) > targetCount:
			groupSize = self.buffer[0].payload.shape[1] / channelCount
			while targetCount > 0:
				# read more samples

				if targetCount >= groupSize - self.prevReadSamples:
					# can read whole packet
					data = self.buffer.popleft()
					newSamples = data.payload.reshape(-1, channelCount)[self.prevReadSamples:]
					self.prevReadSamples = 0 # reset to read next packet from the start
				else:
					# read only part of packet, leave it in the buffer (in order
					# to read the rest of it in the future)
					newSamples = self.buffer[0].payload.reshape(-1, channelCount)[self.prevReadSamples:self.prevReadSamples + targetCount]
					self.prevReadSamples += targetCount # store how much of the packet was already read
				
				# add new samples
				samples = np.append(samples, newSamples)
				targetCount -= len(newSamples)
		
		return samples
	
	def getAvailableAudioSamplesCount(self, channelCount):
		"""Get total number of available samples.

		Args:
			channelCount: Number of channels within a buffer.
		"""
		sampleCount = 0
		for audioData in self.buffer:
			sampleCount += audioData.payload.shape[1] / channelCount
		return sampleCount - self.prevReadSamples

	def processAudio(self, audioSamples, channelCount, scriptOp):
		"""Push audio data into Script CHOP.
		"""
		if audioSamples.dtype == np.int16:
			# RAW, L2 uses np.int16
			audioSamples = (audioSamples / (65536 / 2)).astype(np.float32)
		chopData = np.ascontiguousarray(audioSamples.reshape(-1, channelCount).transpose())
		scriptOp.lock = True
		scriptOp.copyNumpyArray(chopData, baseName='chan')
		scriptOp.rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
		scriptOp.start = ((scriptOp.time.frame - 2) * chopData.shape[1]) + 1
		scriptOp.lock = False

	def handleCalibration(self):
		"""Push hl2ss calibration to TD.
		"""
		calib = {}
		if self.port in (
			hl2ss.StreamPort.RM_VLC_LEFTFRONT,
			hl2ss.StreamPort.RM_VLC_LEFTLEFT,
			hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
			hl2ss.StreamPort.RM_VLC_RIGHTRIGHT
			):
			self.processImage(self.calibration.uv2xy, op('script_img3'))
			self.processImage(self.calibration.undistort_map, op('script_img4'))
			calib = {
				'extrinsics': self.calibration.extrinsics.tolist(),
				'intrinsics': self.calibration.intrinsics.tolist()
			}

		elif self.port == hl2ss.StreamPort.RM_DEPTH_AHAT:
			self.processImage(self.calibration.uv2xy, op('script_img3'))
			self.processImage(self.calibration.undistort_map, op('script_img4'))
			calib = {
				'extrinsics': self.calibration.extrinsics.tolist(),
				'scale': self.calibration.scale.tolist(),
				'alias': self.calibration.alias.tolist(),
				'intrinsics': self.calibration.intrinsics.tolist()
			}

		elif self.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW:
			self.processImage(self.calibration.uv2xy, op('script_img3'))
			self.processImage(self.calibration.undistort_map, op('script_img4'))
			calib = {
				'extrinsics': self.calibration.extrinsics.tolist(),
				'scale': self.calibration.scale.tolist(),
				'intrinsics': self.calibration.intrinsics.tolist()
			}

		elif self.port in (
			hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
			hl2ss.StreamPort.RM_IMU_GYROSCOPE
			):
			calib = {
				'extrinsics': self.calibration.extrinsics.tolist(),
			}

		elif self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			calib = {
				'focal_length': self.calibration.focal_length.tolist(),
				'principal_point': self.calibration.principal_point.tolist(),
				'radial_distortion': self.calibration.radial_distortion.tolist(),
				'tangential_distortion': self.calibration.tangential_distortion.tolist(),
				'projection': self.calibration.projection.tolist(),
				'intrinsics': self.calibration.intrinsics.tolist(),
				'extrinsics': self.calibration.extrinsics.tolist(),
				'intrinsics_mf': self.calibration.intrinsics_mf.tolist(),
				'extrinsics_mf': self.calibration.extrinsics_mf.tolist()
			}

		op('calibration').text = json.dumps(calib, indent=4)

	def GetFlipFlops(self) -> tuple[bool, bool, int]:
		"""Provides correct image transformation
		(based on currect port) for Flip TOP.

		Returns:
			Tuple containing FlipX, FlipY, Flop.
		"""
		if self.port == hl2ss.StreamPort.RM_VLC_LEFTFRONT:
			return (False, False, 2)
		elif self.port == hl2ss.StreamPort.RM_VLC_LEFTLEFT:
			return (False, False, 1)
		elif self.port == hl2ss.StreamPort.RM_VLC_RIGHTFRONT:
			return (False, False, 1)
		elif self.port == hl2ss.StreamPort.RM_VLC_RIGHTRIGHT:
			return (False, False, 2)
		elif self.port == hl2ss.StreamPort.RM_DEPTH_AHAT or self.port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW:
			return (False, True, 0)
		elif self.port == hl2ss.StreamPort.PERSONAL_VIDEO:
			return (False, True, 0)
		else:
			return (False, False, 0)
		
	def GetChopNames(self) -> str:
		"""Provides correct channel names for Rename CHOP.
		"""
		if self.port in (
			hl2ss.StreamPort.RM_IMU_ACCELEROMETER,
			hl2ss.StreamPort.RM_IMU_GYROSCOPE,
			hl2ss.StreamPort.RM_IMU_MAGNETOMETER
			):
			return 'x y z temperature'
		else:
			return ''
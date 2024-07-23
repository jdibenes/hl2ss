class Hl2ssFramePickerExt():
	"""
	Hl2ssFramePickerExt description
	"""
	def __init__(self, ownerComp):
		self.ownerComp = ownerComp
		self.lastPresentedFramestamp = 0
		self.data = None

		# initial cleanup
		op('metadata').text = op('default_metadata').text
	
	@property
	def MainComp(self):
		return self.ownerComp.par.Maincomp.eval()

	@property
	def Data(self):
		return self.data

	def OnTdFrame(self):
		"""Try to process next frame. This method needs to run on each TD frame.

		Returns:
		    bool: Success of providing new data.
		"""
		if self.MainComp is not None:
			return self.MainComp.GetAndPresentFrame(extObj=self)
		return False
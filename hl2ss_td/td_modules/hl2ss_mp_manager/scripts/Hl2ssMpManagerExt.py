import multiprocessing as mp
import os
import time

class Hl2ssMpManagerExt:
	"""
	Provides common multiprocessing manager for hl2ss comps.
	"""
	def __init__(self, ownerComp):
		self.ownerComp = ownerComp
		mp.set_executable( os.path.join(app.binFolder, 'pythonw.exe') )
		self.Manager = mp.Manager()
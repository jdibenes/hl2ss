def onExit():
	parent().CloseStream()

def onFrameStart(frame):
	parent().OnTdFrame()
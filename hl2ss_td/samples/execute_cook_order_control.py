def onFrameStart(frame):
	nodes = [op('hl2ss_core_1'), op('hl2ss_core_2'), op('hl2ss_core_3')]
	rets = []
	for node in nodes:
		ret = node.OnTdFrame()
		rets.append(ret)
	if any(rets):
		# some component has new data, lets do processing
		return
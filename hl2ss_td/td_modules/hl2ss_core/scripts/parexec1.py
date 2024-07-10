def onValueChange(par, prev):
    if par.name == 'Stream':
        if par.eval():
            parent().OpenStream()
        else:
            parent().CloseStream()


def onPulse(par):
    if par.name == 'Getcalib':
        parent().GetCalibration()

#------------------------------------------------------------------------------
# Demo Helpers
#------------------------------------------------------------------------------

class pose_printer:
    def __init__(self, period):
        self._period = period
        self._count = 0

    def push(self, timestamp, pose):
        self._count += 1
        if (self._count >= self._period):
            self._count = 0
            if (pose is not None):
                print('Pose at time {ts}'.format(ts=timestamp))
                print(pose)


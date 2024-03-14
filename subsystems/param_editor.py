import sys
sys.path.append('.')
import constants

from wpilib import SmartDashboard
from wpimath.controller import PIDController
from commands2 import Subsystem

pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber


class ParamEditor(Subsystem):

    def __init__(self, pid_controller: PIDController):
        super().__init__()
        self.pid = pid_controller
        self.published = False

    def publish_to_sd(self):
        pn('testP', self.pid.getP())
        pn('testI', self.pid.getI())
        pn('testD', self.pid.getD())

    def read_from_sd(self):
        p = gn('testP', 0)
        i = gn('testI', 0)
        d = gn('testD', 0)
        self.pid.setP(p)
        self.pid.setI(i)
        self.pid.setD(d)

    def periodic(self):
        if self.published is False:
            self.publish_to_sd()
            self.published = True
        self.read_from_sd()


if __name__ == '__main__':
    pe = ParamEditor()
    pe.publish_to_sd()
    pe.read_from_sd()
    print('Done')

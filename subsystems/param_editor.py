import sys
sys.path.append('.')
import constants

from wpilib import SmartDashboard
from commands2 import Subsystem

pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber


class ParamEditor(Subsystem):

    def __init__(self):
        super().__init__()

    def publish_to_sd(self):
        print('Sending to SmartDashboard')

    def read_from_sd(self):
        d = gn('str_d', 0)
        constants.RobotPIDConstants.straight_drive_pid[2] = d
        print('Reading from SmartDashboard')


if __name__ == '__main__':
    pe = ParamEditor()
    pe.publish_to_sd()
    pe.read_from_sd()
    print('Done')

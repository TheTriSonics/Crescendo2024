# This file should be deleted.

from commands2 import Command
from wpilib import SmartDashboard, Timer
from wpimath.controller import PIDController
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro


class Rotate(Command):
    def __init__(self, drive: Drivetrain, gyro: Gyro, targetHeading):
        super().__init__()
        self.drive = drive
        self.gyro = gyro
        self.targetHeading = targetHeading
        self.timer = Timer()

    def initialize(self):
        self.drive.defcmd.desired_heading = self.targetHeading
        self.timer.restart()
        pass

    def execute(self):
        pass

    def end(self, i):
        print('rotate done')
        pass

    def isFinished(self):
        pose = self.drive.getPose()
        currRot = pose.rotation().degrees()
        deltaRot = abs(currRot - self.targetHeading)
        # SmartDashboard.putNumber('dtp err', deltaRot)
        if deltaRot < 4:
            return True
        if self.timer.hasElapsed(3.0):
            return True
        return False

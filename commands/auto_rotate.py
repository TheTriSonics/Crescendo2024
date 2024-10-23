from commands2 import Command
from wpilib import SmartDashboard, Timer
from wpimath.controller import PIDController
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro


class AutoRotate(Command):
    def __init__(self, drive: Drivetrain, gyro: Gyro, targetHeading):
        super().__init__()
        self.drivetrain = drive
        self.gyro = gyro
        self.targetHeading = targetHeading
        self.timer = Timer()

    def initialize(self):
        print(f"rotate started with heading = {self.targetHeading}")
        self.drivetrain.set_lock_heading(self.targetHeading)
        self.timer.restart()
        pass

    def execute(self):
        
        pass

    def end(self, i):
        print('rotate done')
        pass

    def isFinished(self):
        pose = self.drivetrain.getPose()
        currRot = pose.rotation().degrees()
        deltaRot = abs(currRot - self.targetHeading)
        if deltaRot < 4:
            return True
        if self.timer.hasElapsed(3.0):
            return True
        return False

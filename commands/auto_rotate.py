from commands2 import Command
from wpilib import SmartDashboard, Timer
from wpimath.controller import PIDController
from misc import angle_offset
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro


class AutoRotate(Command):
    def __init__(self, drive: Drivetrain, gyro: Gyro, targetHeading):
        super().__init__()
        self.drivetrain = drive
        self.gyro = gyro
        self.targetHeading = targetHeading
        self.timer = Timer()
        self.offset = 0
        self.rotPID = PIDController(0.05, 0, 0)
        self.addRequirements(drive)

    def initialize(self):
        print(f"rotate started with heading = {self.targetHeading}")
        self.timer.restart()
        pass

    def execute(self):
        pose = self.drivetrain.getPose()
        currRot = pose.rotation().degrees()
        self.offset = angle_offset(self.targetHeading, currRot)
        rotPower = self.rotPID.calculate(self.offset, 0)
        self.drivetrain.drive(0, 0, rotPower)


        pass

    def end(self, i):
        self.drivetrain.drive(0, 0, 0)
        print('rotate done')
        pass

    def isFinished(self):
        if abs(self.offset) < 4:
            return True
        if self.timer.hasElapsed(3.0):
            return True
        return False

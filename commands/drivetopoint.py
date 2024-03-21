from wpilib import SmartDashboard
from commands2 import Command
from wpimath.controller import PIDController
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro

class DriveToPoint(Command):
    def __init__(self, drive: Drivetrain, gyro: Gyro, x, y, targetHeading):
        super().__init__()
        self.drive = drive
        self.gyro = gyro
        self.x = x
        self.y = y
        self.targetHeading = targetHeading
        p, i, d = 3, 0, 0
        self.xPID = PIDController(p, i, d)
        self.yPID = PIDController(p, i, d)
        self.rotPID = PIDController(0.07, 0, 0.004)
        self.addRequirements(self.drive)

    def initialize(self):
        pass

    def execute(self):
        pose = self.drive.getPose()
        curr_yaw = self.gyro.get_yaw()
        currx = pose.X()
        curry = pose.Y()
        xPower = self.xPID.calculate(currx, self.x)
        yPower = self.yPID.calculate(curry, self.y)
        rotPower = self.rotPID.calculate(curr_yaw, self.targetHeading)
        pn = SmartDashboard.putNumber
        pn('dtp/xPower', xPower)
        pn('dtp/yPower', yPower)
        pn('dtp/rotPower', rotPower)
        self.drive.drive(xPower, yPower, rotPower)

    def end(self, i):
        print('dtp done')
        pass

    def isFinished(self):
        from math import sqrt
        pose = self.drive.getPose()
        currx = pose.X()
        curry = pose.Y()
        deltax = currx - self.x
        deltay = curry - self.y
        total_distance = sqrt(deltax*deltax + deltay*deltay)
        if total_distance < 5/100:
            return True
        return False

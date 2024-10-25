from wpilib import SmartDashboard, Timer
from commands2 import Command
from wpimath.controller import PIDController
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro

class DriveToPoint(Command):
    def __init__(self, drive: Drivetrain, gyro: Gyro, x, y, targetHeading):
        super().__init__()
        self.timer = Timer()
        self.drivetrain = drive
        self.gyro = gyro
        if x is None:
            self.x = self.drivetrain.getPose().X()
        else:
            self.x = x
        if y is None:
            self.y = self.drivetrain.getPose().Y()
        else:
            self.y = y
        self.targetHeading = targetHeading
        p, i, d = 3, 0, 0
        self.xPID = PIDController(p, i, d)
        self.yPID = PIDController(p, i, d)
        self.rotPID = PIDController(0.07, 0, 0.004)
        self.addRequirements(self.drivetrain)

    def initialize(self):
        self.timer.restart()
        pass

    def execute(self):
        print(f"DTP target x = {self.x}, y = {self.y}, heading = {self.targetHeading}")
        pose = self.drivetrain.getPose()
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
        self.drivetrain.drive(xPower, yPower, rotPower)

    def end(self, i):
        print('dtp done')
        pass

    def isFinished(self):
        if self.timer.hasElapsed(3.0):
            return True
        from math import sqrt
        pose = self.drivetrain.getPose()
        currx = pose.X()
        curry = pose.Y()
        deltax = currx - self.x
        deltay = curry - self.y
        total_distance = sqrt(deltax*deltax + deltay*deltay)
        pose = self.drivetrain.getPose()
        currRot = pose.rotation().degrees()
        deltaRot = abs(currRot - self.targetHeading)
        if deltaRot < 4 and total_distance < 5/100:
            return True
        return False

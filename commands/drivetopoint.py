from math import sqrt
from wpilib import SmartDashboard, Timer
from commands2 import Command
from wpimath.controller import PIDController
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro
from misc import angle_offset

class DriveToPoint(Command):
    def __init__(self, drive: Drivetrain, gyro: Gyro, x = None, y = None, targetHeading = None):
        super().__init__()
        self.timer = Timer()
        self.drivetrain = drive
        self.gyro = gyro
        self.target_x = x
        self.target_y = y
        self.target_Heading = targetHeading
        p, i, d = 3, 0, 0
        self.xPID = PIDController(p, i, d)
        self.yPID = PIDController(p, i, d)
        self.rotPID = PIDController(0.06, 0, 0)
        self.rotPID.enableContinuousInput(-180,180)
        self.addRequirements(self.drivetrain)
        self.offset = 0
        self.total_distance = 100
        self.distance_target = 5/100
        self.rotational_target = 4


    def initialize(self):
        self.timer.restart()
        if self.target_x is None:
            self.target_x = self.drivetrain.getPose().X()
            print(f"X target was blank, keeping x position at {self.target_x}")
        if self.target_y is None:
            self.target_y = self.drivetrain.getPose().Y()
            print(f"Y target was blank, keeping x position at {self.target_y}")
        if self.target_Heading is None:
            self.target_Heading = self.drivetrain.getPose().rotation().degrees
            print(f"DTP target heading was blank, keeping heading at {self.target_Heading}")
        print(f"DTP target x = {self.target_x}, y = {self.target_y}, heading = {self.target_Heading}")
        pass

    def execute(self):
        pose = self.drivetrain.getPose()
        curr_yaw = self.gyro.get_yaw()
        currx = pose.X()
        curry = pose.Y()
        xPower = self.xPID.calculate(currx, self.target_x)
        yPower = self.yPID.calculate(curry, self.target_y)
        deltax = currx - self.target_x
        deltay = curry - self.target_y
        self.total_distance = sqrt(deltax*deltax + deltay*deltay)
        self.offset = angle_offset(curr_yaw, self.target_Heading)
        rotPower = -self.rotPID.calculate(self.offset, 0)
        """
        if self.total_distance < self.distance_target:
            xPower = 0
            yPower = 0
        if abs(self.offset) < self.rotational_target:
            rotPower = 0
        """
        pn = SmartDashboard.putNumber
        pn("dtp/offset angle", self.offset)
        pn('dtp/xPower', xPower)
        pn('dtp/yPower', yPower)
        pn('dtp/rotPower', rotPower)
        self.drivetrain.drive(xPower, yPower, rotPower)
        print(f"Offset = {self.offset} and Distance = {self.total_distance}")


    def end(self, i):
        print('dtp done')
        pass

    def isFinished(self):
        if self.timer.hasElapsed(3.0):
            return True
        if abs(self.offset) < self.rotational_target and self.total_distance < self.distance_target:
            return True
        return False

from commands2 import Command
from time import time
from subsystems.drivetrain import Drivetrain


class DriveForTime(Command):

    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drive = drive
        self.power = 100
        self.addRequirements(drive)

    def initialize(self):
        self.start = time()

    def execute(self):
        self.drive.drive(self.power, 0, 0, False, 0.02)

    def end(self, i):
        self.drive.lockWheels()
        pass

    def isFinished(self):
        now = time()
        return (now - self.start) > 2

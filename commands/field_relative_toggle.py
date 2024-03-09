# Command to toggle the fieldRelative boolean in the drivetrain subsystem
from wpilib import Timer
from commands2 import Command

from subsystems.drivetrain import Drivetrain


class FieldRelativeToggle(Command):

    def __init__(self, drive: Drivetrain):
        self.timer = Timer()
        self.drive = drive

    def initialize(self):
        self.drive.toggleFieldRelative()

    def execute(self):
        pass

    def end(self, isInterrupted: bool):
        pass

    def isFinished(self) -> bool:
        return True

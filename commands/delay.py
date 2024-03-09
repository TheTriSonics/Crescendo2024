from wpilib import Timer
from commands2 import Command


class Delay(Command):

    def __init__(self, seconds=1):
        self.timer = Timer()
        self.seconds = seconds

    def initialize(self):
        self.timer.restart()

    def execute(self):
        pass

    def end(self, isInterrupted):
        pass

    def isFinished(self):
        return self.timer.hasElapsed(self.seconds)

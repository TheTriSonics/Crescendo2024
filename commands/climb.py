from commands2 import Command
from controllers.driver import DriverController

class Climb(Command):
    def __init__(self, climber, controller: DriverController):
        super().__init__()
        self.climber = climber
        self.controller = controller
        self.addRequirements(climber)

    def execute(self):
        self.climber.set_speed(self.controller.get_climber_trigger()*0.25)

    def end(self, interrupted: bool):
        self.climber.set_speed(0)

    def isFinished(self):
        return False
from wpilib import Joystick
from enum import Enum
from commands2 import Subsystem, Command
from rev import CANSparkMax


class Diverter(Subsystem):
    class Direction(Enum):
        SHOOTER = 1
        AMP = -1

    def __init__(self, motor_id):
        super().__init__()
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)
        self.direction = None

    def set_amp(self):
        self.direction = Diverter.Direction.AMP

    def set_shooter(self):
        self.direction = Diverter.Direction.SHOOTER

    def set_direction(self, direction):
        self.motor.set(0.5)  # Set motor speed to 50% power
        self.direction = direction
        self.motor.set(direction.value)

    def go_shooter(self):
        self.set_direction(Diverter.Direction.SHOOTER)

    def go_amp(self):
        self.set_direction(Diverter.Direction.AMP)

    def halt(self):
        self.motor.stopMotor()


class DiverterDefaultCommand(Command):

    def __init__(self, diverter: Diverter, controller: Joystick):
        self.diverter = diverter
        self.controller = controller

    def execute(self):
        if self.controller.getRawButton(3):
            self.diverter.go_shooter()
        elif self.controller.getRawButton(4):
            self.diverter.go_amp()
        else:
            self.diverter.halt()

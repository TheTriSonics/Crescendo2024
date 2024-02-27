from wpilib import Joystick
from enum import Enum
from commands2 import Subsystem, Command
from rev import CANSparkMax
from phoenix6.hardware import TalonFX

from constants import RobotMotorMap as RMM

#TODO: Make work
class Amp(Subsystem):
    class Direction(Enum):
        SHOOTER = 1
        AMP = -1

    def __init__(self):
        super().__init__()
        self.feed_motor = CANSparkMax(RMM.amp_feed_motor, CANSparkMax.MotorType.kBrushless)

        self.lift_motor = TalonFX(RMM.amp_lift_motor, "canivore")

        self.direction = None

    def set_amp(self):
        self.direction = Amp.Direction.AMP

    def set_shooter(self):
        self.direction = Amp.Direction.SHOOTER

    def set_direction(self, direction):
        self.motor.set(0.5)  # Set motor speed to 50% power
        self.direction = direction
        self.motor.set(direction.value)

    def go_shooter(self):
        self.set_direction(Amp.Direction.SHOOTER)

    def go_amp(self):
        self.set_direction(Amp.Direction.AMP)

    def halt(self):
        self.motor.stopMotor()


class AmpDefaultCommand(Command):

    def __init__(self, amp: Amp, controller: Joystick):
        self.amp = amp
        self.controller = controller

    def execute(self):
        if self.controller.getRawButton(3):
            self.amp.go_shooter()
        elif self.controller.getRawButton(4):
            self.amp.go_amp()
        else:
            self.amp.halt()


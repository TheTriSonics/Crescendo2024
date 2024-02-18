from commands2 import Subsystem, Command
from rev import CANSparkMax, CANSparkLowLevel

from constants import RobotMap
from subsystems.photoeyes import PhotoEyes
from controllers.commander import CommanderController


class Intake(Subsystem):
    feed_motors = CANSparkMax(RobotMap.intake_feed,
                              CANSparkLowLevel.MotorType.kBrushless)
    lift_motor = CANSparkMax(RobotMap.intake_lift,
                             CANSparkLowLevel.MotorType.kBrushless)

    def __init__(self, controller: CommanderController,
                 photoeyes: PhotoEyes) -> None:
        super().__init__()
        self.controller = controller
        self.photoeyes = photoeyes
        defcmd = IntakeDefaultCommand(self, self.controller,
                                      self.photoeyes)
        self.setDefaultCommand(defcmd)

    def feed(self, speed: float) -> None:
        self.feed_motors.set(speed)

    # The motor will turn off when the limit switches wired into the controller
    # tell it to. So, we only need to drive the motor in the direction we want
    # "forever" and it'll stop when needed.
    def lift(self, speed: float) -> None:
        self.lift_motor.set(speed)


class IntakeDefaultCommand(Command):

    def __init__(self, intake: Intake, controller: CommanderController,
                 photoeyes: PhotoEyes) -> None:
        super().__init__()

        self.intake = intake
        self.photoeyes = photoeyes
        self.controller = controller
        self.lift_speed = -0.5  # Default to raising up
        self.addRequirements(intake)

    def execute(self) -> None:

        intake_on = self.controller.get_intake_on()
        reverse_down = self.controller.get_intake_reverse()
        override_down = self.controller.get_intake_override()
        eye_blocked = self.photoeyes.intake_full()

        intake_speed = 0
        if intake_on and eye_blocked is False:
            intake_speed = 0.5
            self.lift_speed = -0.5
        elif intake_on and override_down:
            intake_speed = 0.5
            self.lift_speed = -0.5
        elif reverse_down and override_down:
            intake_speed = -0.5
            self.lift_speed = 0.5

        self.intake.feed(intake_speed)
        self.intake.lift(self.lift_speed)

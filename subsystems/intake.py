from wpilib import Joystick
from commands2 import Subsystem, Command
from rev import CANSparkMax, CANSparkLowLevel

from constants import RobotMap
from subsystems.photoeyes import PhotoEyes


class Intake(Subsystem):

    feed_motors: CANSparkMax
    tilt_motor: CANSparkMax

    def __init__(self, controller: Joystick, photoeyes: PhotoEyes) -> None:
        super().__init__()

        self.feed_motors = CANSparkMax(RobotMap.intake_feed,
                                       CANSparkLowLevel.MotorType.kBrushless)
        self.tilt_motor = CANSparkMax(RobotMap.intake_tilt,
                                      CANSparkLowLevel.MotorType.kBrushless)

        self.controller = controller
        self.photoeyes = photoeyes
        defcmd = IntakeDefaultCommand(self, self.controller, self.photoeyes)
        self.setDefaultCommand(defcmd)

    def feed(self, speed: float) -> None:
        self.feed_motors.set(speed)


class IntakeDefaultCommand(Command):

    def __init__(self, controller: Joystick, intake: Intake,
                 photoeyes: PhotoEyes) -> None:
        super().__init__()

        self.intake = intake
        self.photoeyes = photoeyes
        self.controller = controller
        self.addRequirements(intake)

    def execute(self) -> None:
        intake_speed = 0

        button_down = self.controller.getRawButton(1)
        reverse_down = self.controller.getRawButton(2)
        override_down = self.controller.getRawButton(20)
        eye_blocked = self.photoeyes.intake_full()

        if button_down and eye_blocked is False:
            intake_speed = 0.5
        elif button_down and override_down:
            intake_speed = 0.5
        elif reverse_down and override_down:
            intake_speed = -0.5

        self.intake.feed(intake_speed)


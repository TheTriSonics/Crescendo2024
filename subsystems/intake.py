from rev import CANSparkMax, CANSparkLowLevel
from constants import RobotMap
from commands2 import Subsystem, Command
from subsystems.photoeyes import PhotoEyes
from controllers.commander import CommanderController


class Intake(Subsystem):

    def __init__(self, controller: CommanderController,
                 photoeyes: PhotoEyes) -> None:
        super().__init__()
        self.controller = controller
        self.photoeyes = photoeyes
        self.feed_motors = CANSparkMax(RobotMap.intake_feed,
                                       CANSparkLowLevel.MotorType.kBrushless)
        self.divert_motor = CANSparkMax(RobotMap.intake_divert,
                                        CANSparkLowLevel.MotorType.kBrushless)
        defcmd = IntakeDefaultCommand(self, self.controller,
                                      self.photoeyes)
        self.setDefaultCommand(defcmd)

    def feed(self, speed: float) -> None:
        self.feed_motors.set(speed)

    def diverter(self, speed: float) -> None:
        self.divert_motor.set(speed)


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
        # Pattern: 1) Gather Info

        # Read all of the buttons we need to take into account to make our
        # decision
        intake_on = self.controller.get_intake_on()
        reverse_down = self.controller.get_intake_reverse()
        override_down = self.controller.get_intake_override()
        eye_blocked = self.photoeyes.intake_full()

        divert_shooter = self.controller.get_diverter_shooter()
        divert_amp = self.controller.get_diverter_amp()

        # Pattern:  2) Make decision
        # Set up a default value for if no conditions match, or no buttons are
        # pressed.
        intake_speed = 0
        # Now step through combinations to see what we should do
        if intake_on and eye_blocked is False:
            intake_speed = 0.5
        elif intake_on and override_down:
            intake_speed = 0.5
        elif reverse_down and override_down:
            intake_speed = -0.5

        divert_speed = 0
        if divert_shooter:
            divert_speed = 0.5
        elif divert_amp:
            divert_speed = -0.5

        # Pattern: 3) Execute decision
        # Now commit some values to the physical subsystem.
        self.intake.feed(intake_speed)
        self.intake.diverter(divert_speed)

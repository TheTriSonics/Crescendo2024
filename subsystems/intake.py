from rev import CANSparkLowLevel
from rev import CANSparkMax
from wpilib import Joystick, SmartDashboard
from constants import RobotMotorMap as RMM, RobotSensorMap as RSM
from commands2 import Subsystem, Command
from subsystems.photoeyes import Photoeyes
from controllers.commander import CommanderController

# tilt_encoder_setpoint_down = 0.42
tilt_encoder_setpoint_down = 0.43
tilt_encoder_setpoint_up = 0.335
tilt_encoder_error_margin = 0.005


class Intake(Subsystem):

    def __init__(self, controller: CommanderController,
                 photoeyes: Photoeyes) -> None:
        super().__init__()
        self.controller = controller
        self.photoeyes = photoeyes
        self.feeding = False
        self.reversing = False

        self.feed_motors = CANSparkMax(RMM.intake_motor_feed,
                                       CANSparkLowLevel.MotorType.kBrushed)

        # Wherever the lift is on boot is good enough for us right now.
        self.tilt_setpoint = tilt_encoder_setpoint_up

    def feed(self) -> None:
        self.feeding = True
        self.reversing = False
        self.feed_motors.set(1)

    def reverse(self) -> None:
        self.reversing = True
        self.feeding = False
        self.feed_motors.set(-1)

    def halt(self) -> None:
        self.reversing = False
        self.feeding = False
        self.feed_motors.set(0)

    def tilt_up(self) -> None:
        self.tilt_setpoint = tilt_encoder_setpoint_up

    def tilt_down(self) -> None:
        self.tilt_setpoint = tilt_encoder_setpoint_down

    def wanted_up(self) -> bool:
        return self.tilt_setpoint == tilt_encoder_setpoint_up

    def wanted_down(self) -> bool:
        return self.tilt_setpoint == tilt_encoder_setpoint_down

    def is_up(self) -> bool:
        curr = self.tilt_encoder.getAbsolutePosition()
        setpoint = tilt_encoder_setpoint_up
        return abs(curr - setpoint) < tilt_encoder_error_margin

    def is_down(self) -> bool:
        curr = self.tilt_encoder.getAbsolutePosition()
        setpoint = tilt_encoder_setpoint_down
        return abs(curr - setpoint) < tilt_encoder_error_margin

    def is_loaded(self) -> bool:
        # If the intake is loaded this should return True
        return self.photoeyes.get_intake_loaded()

    # The scheduler will call this method every 20ms and it will drive the
    # lift to the desired position using our PID controller
    def periodic(self) -> None:
        pass

class IntakeDefaultCommand(Command):

    def __init__(self, intake: Intake, controller: CommanderController,
                 photoeyes: Photoeyes) -> None:
        super().__init__()

        self.intake = intake
        self.photoeyes = photoeyes
        self.controller = Joystick(1)
        self.addRequirements(intake)
        self.feed = False

    def execute(self) -> None:
        intake_speed = 0

        if self.controller.getRawButton(1):
            self.intake.reverse()

        if self.controller.getRawButton(2):
            self.intake.tilt_up()
        elif self.controller.getRawButton(3):
            self.intake.tilt_down()
        SmartDashboard.putNumber('intake/intake_speed', intake_speed)

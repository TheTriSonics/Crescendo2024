import rev
import wpilib
from rev import CANSparkLowLevel
from rev import CANSparkMax
from misc import is_sim
from wpilib import DutyCycleEncoder, Joystick, SmartDashboard
from constants import RobotMotorMap as RMM, RobotSensorMap as RSM
from commands2 import Subsystem, Command
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
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

        # defcmd = IntakeDefaultCommand(self, self.controller,
        #                               self.photoeyes)
        # self.setDefaultCommand(defcmd)

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
        # Get the encoder reading, a number value.
        pass

    # def getSimulatedPosition(self):
    #     return self.simulated_position

    def simulationPeriodic(self) -> None:
        # current_pos = self.tilt_encoder.getAbsolutePosition()
        # if self.tilt_setpoint > current_pos:
        #     self.tilt_encoder._position = current_pos + 1
        # else:
        #     self.tilt_encoder._position = current_pos - 1
        # print(current_pos, self.tilt_setpoint)
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
        # Pattern: 1) Gather Info

        # Read all of the buttons we need to take into account to make our
        # decision
        # intake_on = self.controller.get_intake_ready()
        # reverse_down = self.controller.get_override_intake_roller_out()
        # override_down = self.controller.get_override_intake_tilt_down()
        # eye_blocked = self.photoeyes.get_intake_loaded()

        # tilt_down = self.controller.get_override_intake_tilt_down()
        # tilt_up = self.controller.get_override_intake_tilt_up()

        # Pattern:  2) Make decision
        # Set up a default value for if no conditions match, or no buttons are
        # pressed.
        intake_speed = 0

        # Now step through combinations to see what we should do
        # if abs(self.controller.getRawAxis(1)) >= 0.04:
        #     intake_speed = self.controller.getRawAxis(1)
        # else:
        #     intake_speed = 0

        # tilt_speed = 0
        # if abs(self.controller.getRawAxis(5)) >= 0.04:
        #     tilt_speed = self.controller.getRawAxis(5) * 0.2
        # else:
        #     tilt_speed = 0

        if self.controller.getRawButton(1):
            self.intake.reverse()

        if self.controller.getRawButton(2):
            self.intake.tilt_up()
        elif self.controller.getRawButton(3):
            self.intake.tilt_down()

        # if intake_on and eye_blocked is False:
        #     intake_speed = 0.5
        # elif intake_on and override_down:
        #     intake_speed = 0.5
        # elif reverse_down and override_down:
        #     intake_speed = -0.5

        # if tilt_up:
        #     self.tilt_setpoint = tilt_encoder_setpoint_up
        # elif tilt_down:
        #     self.tilt_setpoint = tilt_encoder_setpoint_down

        # Pattern: 3) Execute decision
        # Now commit some values to the physical subsystem.
        # self.intake.feed()
        # self.intake.feed_motors.set(intake_speed)
        # self.intake.tilt_motor.set(tilt_speed)
        SmartDashboard.putNumber('intake/intake_speed', intake_speed)

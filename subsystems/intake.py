import rev
import wpilib
from rev import CANSparkLowLevel
from misc import is_sim
from wpilib import SmartDashboard
from constants import RobotMotorMap as RMM
from commands2 import Subsystem, Command
from wpimath.controller import PIDController
from subsystems.photoeyes import PhotoEyes
from controllers.commander import CommanderController

# TODO: Sort these actual values out with real hardware
tilt_encoder_setpoint_down = 0
tilt_encoder_setpoint_up = 200
tilt_encoder_error_margin = 5



if is_sim():
    class SparkMaxAbsoluteEncoder:

        def __init__(self) -> None:
            self._position = 0

        def getPosition(self):
            return self._position

    class CANSparkMax(wpilib.Spark):
        IdleMode = rev.CANSparkMax.IdleMode

        def __init__(self, channel: int, ignored) -> None:
            super().__init__(channel)
            self._encoder = SparkMaxAbsoluteEncoder()

        def getAbsoluteEncoder(self):
            return self._encoder

        def setIdleMode(self, mode):
            pass

else:
    import rev

    CANSparkMax = rev.CANSparkMax


class Intake(Subsystem):

    def __init__(self, controller: CommanderController,
                 photoeyes: PhotoEyes) -> None:
        super().__init__()
        self.controller = controller
        self.photoeyes = photoeyes

        self.feed_motors = CANSparkMax(RMM.intake_feed,
                                       CANSparkLowLevel.MotorType.kBrushed)
        self.tilt_motor = CANSparkMax(RMM.intake_tilt,
                                      CANSparkLowLevel.MotorType.kBrushed)

        # Set the tilt_motor to brake mode
        self.tilt_pid = PIDController(0.1, 0, 0)
        self.tilt_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.tilt_encoder = self.tilt_motor.getAbsoluteEncoder()

        # Wherever the lift is on boot is good enough for us right now.
        self.tilt_setpoint = self.tilt_encoder.getPosition()
        defcmd = IntakeDefaultCommand(self, self.controller,
                                      self.photoeyes)
        self.setDefaultCommand(defcmd)

    def feed(self, speed: float) -> None:
        self.feed_motors.set(speed)

    # The scheduler will call this method every 20ms and it will drive the
    # lift to the desired position using our PID controller
    def periodic(self) -> None:
        # Get the encoder reading, a number value.
        current_pos = self.tilt_encoder.getPosition()
        # Set default output to no power, so unless we change this
        # the tilt motor won't be run.
        output = 0
        # Determie if we're too far away from the setpoint to stop
        if abs(current_pos - self.tilt_setpoint) > tilt_encoder_error_margin:
            output = self.tilt_pid.calculate(current_pos, self.tilt_setpoint)
        # Now give whatever value we decided on to the tilt motors.
        self.tilt_motor.set(output)
        # Display the subsystem status on a dashboard
        SmartDashboard.putNumber('intake/tilt_setpoint', self.tilt_setpoint)
        SmartDashboard.putNumber('intake/tilt_current',
                                 self.tilt_encoder.getPosition())
        SmartDashboard.putNumber('intake/tilt_output', output)

    def getSimulatedPosition(self):
        return self.simulated_position

    def simulationPeriodic(self) -> None:
        current_pos = self.tilt_encoder.getPosition()
        if self.tilt_setpoint > current_pos:
            self.tilt_encoder._position = current_pos + 1
        else:
            self.tilt_encoder._position = current_pos - 1
        # print(current_pos, self.tilt_setpoint)
        pass


class IntakeDefaultCommand(Command):

    def __init__(self, intake: Intake, controller: CommanderController,
                 photoeyes: PhotoEyes) -> None:
        super().__init__()

        self.intake = intake
        self.photoeyes = photoeyes
        self.controller = controller
        self.addRequirements(intake)

    def execute(self) -> None:
        # Pattern: 1) Gather Info

        # Read all of the buttons we need to take into account to make our
        # decision
        intake_on = self.controller.get_intake_ready()
        reverse_down = self.controller.get_intake_eject()
        override_down = self.controller.get_override_intake_tilt_down()
        eye_blocked = self.photoeyes.intake_loaded()

        tilt_down = self.controller.get_intake_ready()
        tilt_up = self.controller.get_intake_eject()

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

        if tilt_up:
            self.tilt_setpoint = tilt_encoder_setpoint_up
        elif tilt_down:
            self.tilt_setpoint = tilt_encoder_setpoint_down

        # Pattern: 3) Execute decision
        # Now commit some values to the physical subsystem.
        self.intake.feed(intake_speed)
        SmartDashboard.putNumber('intake/intake_speed', intake_speed)
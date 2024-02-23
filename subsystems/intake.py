import wpilib
from rev import CANSparkMax, CANSparkLowLevel
from wpilib import SmartDashboard
from constants import RobotMap
from commands2 import Subsystem, Command
from wpimath.controller import PIDController
from subsystems.photoeyes import PhotoEyes
from controllers.commander import CommanderController

# TODO: Sort these actual values out with real hardware
tilt_encoder_setpoint_down = 0
tilt_encoder_setpoint_up = 200
tilt_encoder_error_margin = 5


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()


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
        self.tilt_motor = CANSparkMax(RobotMap.intake_tilt,
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

    def diverter(self, speed: float) -> None:
        self.divert_motor.set(speed)

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

    def simulationPeriodic(self) -> None:
        current_pos = self.tilt_encoder.getPosition()
        # setDistance doesn't exist on the sparkmax encoder
        # if self.tilt_setpoint > current_pos:
        #     self.tilt_encoder.setDistance(current_pos + 1)
        # else:
        #     self.tilt_encoder.setDistance(current_pos - 1)
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
        intake_on = self.controller.get_intake_on()
        reverse_down = self.controller.get_intake_reverse()
        override_down = self.controller.get_intake_override()
        eye_blocked = self.photoeyes.intake_full()

        divert_shooter = self.controller.get_diverter_shooter()
        divert_amp = self.controller.get_diverter_amp()

        tilt_down = self.controller.get_tilt_up()
        tilt_up = self.controller.get_tilt_down()

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

        if tilt_up:
            self.tilt_setpoint = tilt_encoder_setpoint_up
        elif tilt_down:
            self.tilt_setpoint = tilt_encoder_setpoint_down

        # Pattern: 3) Execute decision
        # Now commit some values to the physical subsystem.
        self.intake.feed(intake_speed)
        self.intake.diverter(divert_speed)
        SmartDashboard.putNumber('intake/intake_speed', intake_speed)
        SmartDashboard.putNumber('intake/divert_speed', divert_speed)

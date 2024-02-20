from rev import CANSparkMax, CANSparkLowLevel
from wpilib import Encoder
from wpimath.controller import PIDController
from constants import RobotMap
from commands2 import Subsystem, Command
from subsystems.photoeyes import PhotoEyes
from controllers.commander import CommanderController

# TODO: Sort these actual values out with real hardware
encoder_setpoint_down = 0
encoder_setpoint_up = 200


class IntakeTilt(Subsystem):

    def __init__(self, controller: CommanderController) -> None:
        super().__init__()
        self.controller = controller
        self.pid = PIDController(0.1, 0, 0)
        self.tilt_motor = CANSparkMax(RobotMap.intake_tilt,
                                      CANSparkLowLevel.MotorType.kBrushless)
        # Set the tilt_motor to brake mode
        self.tilt_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.tilt_encoder = Encoder(RobotMap.intake_tilt_encoder_a,
                                    RobotMap.intake_tilt_encoder_b)
        self.tilt_encoder.setDistancePerPulse(1)
        # TODO: Determine what the actual startup position
        # for the robot will be. I am assuming it starts in the down position
        # with this code
        self.tilt_encoder.reset()
        self.setpoint = encoder_setpoint_down
        defcmd = IntakeTiltDefaultCommand(self, self.controller)
        self.setDefaultCommand(defcmd)

    # The scheduler will call this method every 20ms and it will drive the
    # lift to the desired position using our PID controller
    def periodic(self) -> None:
        current_pos = self.tilt_encoder.getDistance()
        output = 0
        if abs(current_pos - self.setpoint) > 5:
            output = self.pid.calculate(current_pos, self.setpoint)
        self.tilt_motor.set(output)


class IntakeTiltDefaultCommand(Command):

    def __init__(self, intake_tilt: IntakeTilt,
                 controller: CommanderController) -> None:
        super().__init__()

        self.intake_tilt = intake_tilt
        self.controller = controller
        self.addRequirements(intake_tilt)

    def execute(self) -> None:
        # Pattern: 1) Gather Info
        # Read all of the buttons we need to take into account to make our
        # decision
        tilt_down = self.controller.get_tilt_up()
        tilt_up = self.controller.get_tilt_down()

        # Pattern:  2) Make decision
        # Set up a default value for if no conditions match, or no buttons are
        # pressed.

        if tilt_up:
            self.intake_tilt.setpoint = encoder_setpoint_up
        elif tilt_down:
            self.intake_tilt.setpoint = encoder_setpoint_down

        # Pattern: 3) Execute decision
        # Here we don't actually do anything; the periodic method will handle
        # driving the subsystem to the proper setpoint.


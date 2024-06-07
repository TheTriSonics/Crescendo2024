
import math
from commands2 import Command
from wpilib import PS4Controller, SmartDashboard
from controllers.driver import DriverController
from subsystems.drivetrain import Drivetrain
from subsystems.gyro import Gyro
from subsystems.intake import Intake
from subsystems.note_tracker import NoteTracker
from wpimath.filter import SlewRateLimiter, LinearFilter
from wpimath.controller import PIDController
from constants import RobotPIDConstants as PIDC

sdbase = 'fakesensors/drivetrain'

pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

kMaxSpeed = 4.5  # m/s
kMaxAngularSpeed = math.pi * 5

curr_note_intake_speed = 2
default_note_intake_speed = 2


class DrivetrainDefaultCommand(Command):
    """
    Default command for the drivetrain.
    """
    def __init__(self, drivetrain: Drivetrain, controller: DriverController,
                 photon: NoteTracker, gyro: Gyro, intake: Intake) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.controller = controller
        self.photon = photon
        self.gyro = gyro
        self.intake = intake
        self.note_pid = PS4Controller(*PIDC.note_tracking_pid)
        self.note_translate_pid = PIDController(*PIDC.note_translate_pid)
        self.speaker_pid = PIDController(*PIDC.speaker_tracking_pid)
        self.straight_drive_pid = PIDController(*PIDC.straight_drive_pid)
        self.straight_drive_pid.setTolerance(2.0)
        # Slew rate limiters to make joystick inputs more gentle
        self.xslew = SlewRateLimiter(5.0)
        self.yslew = SlewRateLimiter(5.0)
        self.rotslew = SlewRateLimiter(2)
        self.note_yaw_filtered = LinearFilter.highPass(0.1, 0.02)
        self.idle_counter = 0
        self.desired_heading = None
        self.addRequirements(drivetrain)

    def initialize(self):
        # Heading is in degrees here
        if self.desired_heading is None:
            self.desired_heading = self._curr_heading()
        self.swapped = False
        self.slow_mode = False

    def note_tracking_on(self):
        self.note_lockon = True

    def note_tracking_off(self):
        self.note_lockon = False

    def speaker_tracking_on(self):
        self.speaker_lockon = True

    def speaker_tracking_off(self):
        self.speaker_lockon = False

    def _curr_heading(self) -> float:
        return self.drivetrain.get_heading_rotation_2d().degrees()

    def flipHeading(self):
        self.desired_heading += 180
        if self.desired_heading > 360:
            self.desired_heading -= 360

    def swapDirection(self):
        self.swapped = not self.swapped

    def lock_heading(self):
        self.desired_heading = self._curr_heading()

    def is_note_tracking(self):
        fake = gb(f'{sdbase}/note_tracking', False)
        return self.drivetrain.note_tracking or fake

    def is_note_visible(self):
        fake = gb(f'{sdbase}/note_visible', False)
        return self.drivetrain.note_visible or fake

    def is_speaker_tracking(self):
        fake = gb(f'{sdbase}/speaker_tracking', False)
        return self.drivetrain.speaker_tracking or fake

    def is_speaker_visible(self):
        fake = gb(f'{sdbase}/speaker_visible', False)
        return self.drivetrain.speaker_visible or fake

    def is_speaker_aimed(self):
        fake = gb(f'{sdbase}/speaker_aimed', False)
        return self.drivetrain.speaker_aimed or fake

    def is_amp_tracking(self):
        fake = gb(f'{sdbase}/amp_tracking', False)
        return self.drivetrain.amp_tracking or fake

    def is_amp_visible(self):
        fake = gb(f'{sdbase}/amp_visible', False)
        return self.drivetrain.amp_visible or fake

    def _get_x_speed(self) -> float:
        master_throttle = self.controller.get_master_throttle()
        xraw = self.controller.get_drive_x()
        xSpeed = self.xslew.calculate(xraw)
        if xraw == 0:
            xSpeed /= 2
        xSpeed *= kMaxSpeed
        if self.swapped:
            xSpeed = -xSpeed
        return xSpeed*master_throttle

    def _get_y_speed(self) -> float:
        master_throttle = self.controller.get_master_throttle()
        yraw = self.controller.get_drive_y()
        ySpeed = self.yslew.calculate(yraw)
        if yraw == 0:
            ySpeed /= 2
        ySpeed *= kMaxSpeed
        if self.swapped:
            ySpeed = -ySpeed
        return ySpeed*master_throttle

    def _get_rotation(self) -> float:
        master_throttle = self.controller.get_master_throttle()
        rotraw = self.controller.get_drive_rot()
        rot = self.rotslew.calculate(rotraw)
        if rotraw == 0:
            rot /= 2
        rot *= kMaxAngularSpeed
        return rot*master_throttle

    def lock_heading_helper(self, rot):
        curr = self.drivetrain.get_heading_rotation_2d().degrees()
        # If the user is commanding rotation set the desired heading to the
        # current heading so if they let off we can use PID to keep the robot
        # driving straight
        if rot != 0:
            self.lock_heading()
        else:
            error = curr - self.desired_heading
            if abs(error) > 1.0:
                rot = self.straight_drive_pid.calculate(curr,
                                                        self.desired_heading)
            else:
                rot = 0
        return rot

    def get_stick_data(self):
        xSpeed = self._get_x_speed()
        ySpeed = self._get_y_speed()
        rot = self._get_rotation()
        # TODO: Move to an InstantCommand
        if self.controller.get_slow_mode():
            self.slow_mode = True
            slow_mode_factor = 1/2
            xSpeed *= slow_mode_factor
            ySpeed *= slow_mode_factor
            rot *= slow_mode_factor
        return xSpeed, ySpeed, rot

    def execute(self) -> None:
        xSpeed, ySpeed, rot = self.get_stick_data()
        self.drivetrain.drive(xSpeed, ySpeed, rot, robot_centric_force=False)


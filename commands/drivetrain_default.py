
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
        self.note_lockon = False
        self.speaker_lockon = False
        self.swapped = False

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

    def execute(self) -> None:
        # Heading is in degrees here
        if self.desired_heading is None:
            self.desired_heading = self._curr_heading()
        curr = self.drivetrain.get_heading_rotation_2d().degrees()
        xraw = self.controller.get_drive_x()
        xSpeed = self.xslew.calculate(xraw)
        if xraw == 0:
            xSpeed /= 2
        xSpeed *= kMaxSpeed

        yraw = self.controller.get_drive_y()
        ySpeed = self.yslew.calculate(yraw)
        if yraw == 0:
            ySpeed /= 2
        ySpeed *= kMaxSpeed

        rotraw = self.controller.get_drive_rot()
        rot = self.rotslew.calculate(rotraw)
        if rotraw == 0:
            rot /= 2
        rot *= kMaxAngularSpeed

        master_throttle = self.controller.get_master_throttle()
        xSpeed *= master_throttle
        ySpeed *= master_throttle
        rot *= master_throttle

        if self.swapped:
            xSpeed = -xSpeed
            ySpeed = -ySpeed

        # If the user is commanding rotation set the desired heading to the
        # current heading so if they let off we can use PID to keep the robot
        # driving straight
        if rot != 0:
            self.lock_heading()
        else:
            error = curr - self.desired_heading
            if abs(error) > 1.0:
                rot = self.straight_drive_pid.calculate(curr, self.desired_heading)
            else:
                rot = 0

        if self.controller.get_yaw_reset():
            forward = 180 if self.drivetrain.shouldFlipPath() else 0
            self.gyro.set_yaw(forward)

        # When in lockon mode, the robot will rotate to face the node
        # that PhotonVision is detecting
        robot_centric_force = False

        self.drivetrain.note_tracking = False
        self.drivetrain.note_visible = False
        if self.note_lockon:
            # TODO: Make sure the note intake command is running when this is happening
            # The trick will be kicking the command on but not letting it go away immediately
            # but also don't start a brand new one if it is already running
            """
            if intake_note.running is False:
                sched = CommandScheduler.getInstance()
                sched.schedule(
                    intake_note.IntakeNote(self.intake, self.shooter, self.amp,
                                           self.photoeyes)
                )
            """
            self.note_tracking = True
            robot_centric_force = True
            pn = SmartDashboard.putNumber
            yaw_raw = self.photon.getYawOffset()
            if yaw_raw is not None:
                self.note_visible = True
                self.current_yaw = self.note_yaw_filtered.calculate(yaw_raw)
                pn('drivetrain/note_tracker/yaw_raw', yaw_raw)
                yaw = self.current_yaw
                pn('drivetrain/note_tracker/yaw', yaw)
            pitch = self.photon.getPitchOffset()
            if yaw_raw is not None:
                # Setpoint was 0 but moved to 2 to try and get the
                # robot from going left of the note
                rot = self.note_pid.calculate(yaw_raw, -2)
                xSpeed = curr_note_intake_speed
                # if abs(yaw_raw) < 1.7:
                #     rot = 0
                # else:
                #     rot = self.note_pid.calculate(yaw_raw, 0)
                #     xSpeed = 0.5
                # else:
                #     if pitch is not None and pitch > 0:
                #         rot = self.note_pid.calculate(yaw, 0)
                #     else:
                #         rot = 0
                #         # Force a translation to center on the note when close
                #         ySpeed = self.note_translate_pid.calculate(yaw, 0)

        self.slow_mode = False
        if self.controller.get_slow_mode():
            self.slow_mode = True
            slow_mode_factor = 1/2
            xSpeed *= slow_mode_factor
            ySpeed *= slow_mode_factor
            rot *= slow_mode_factor

        self.drivetrain.speaker_tracking = False
        self.drivetrain.speaker_visible = False
        self.drivetrain.speaker_aimed = False
        if self.speaker_lockon:
            # TODO: Possible! Check with Nathan -- slow down the drivetrain by setting
            # master_throttle to something like 0.8 or 0.6...
            self.drivetrain.speaker_tracking = True
            fid = 4 if self.drivetrain.is_red_alliance() else 7
            speaker_heading = self.drivetrain.get_fid_heading(fid)
            if speaker_heading is not None:
                self.drivetrain.speaker_visible = True
                if abs(speaker_heading) < 3.0:
                    rot = 0
                    self.drivetrain.speaker_aimed = True
                else:
                    rot = self.speaker_pid.calculate(speaker_heading, 0)

        self.drivetrain.drive(xSpeed, ySpeed, rot,
                              robot_centric_force=robot_centric_force)

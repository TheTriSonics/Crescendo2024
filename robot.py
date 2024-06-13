# All units of length will be in meters unless otherwise noted
import json

from time import time
from math import radians
from wpilib import SmartDashboard, Joystick, DriverStation, Timer, Field2d
from commands2 import (ParallelCommandGroup, TimedCommandRobot,
                       SequentialCommandGroup, InstantCommand)
from commands2.button import JoystickButton
from wpimath.geometry import Rotation2d, Pose2d
from pathplannerlib.auto import PathPlannerAuto, NamedCommands

from commands.amp_score import AmpScore
from commands.delay import Delay
from commands.eject_note import EjectNote
from commands.rotate import Rotate
from commands.set_amp_height import SetAmpHeight
from commands.set_amp_override import SetAmpOverride
from commands.shooter_launch_note import ShooterLaunchNote
from commands.intake_note import IntakeNote
from commands.drivetopoint import DriveToPoint
from commands.amp_load import AmpLoad
from commands.shooter_load import ShooterLoad
from commands.shooter_move import ShooterMove
from commands.drivetrain_handler import TrackerType, DrivetrainHandler
from commands.shooter_launch_note_test import ShooterLaunchNoteTest

# An 'import' or 'import ... as ...' brings in code from our other modules
# but doesn't require it to be interpreted right now. We can use it later,
# and then it will, but because the subsystems have a fair number of
# interdependencies the first time we import them here we're going to do it
# in a way that lets them actually load upon use.  You'll likely end up with
# a circular import position with a different technique.
import subsystems.amp as amp
import subsystems.climber as climber
import subsystems.gyro as gyro
import subsystems.intake as intake
import subsystems.param_editor as param_editor
import subsystems.shooter as shooter
import subsystems.photoeyes as photoeyes
import subsystems.drivetrain as drivetrain
import subsystems.speaker_tracker as speaker_tracker
import subsystems.note_tracker as note_tracker
import subsystems.leds as leds
import subsystems.auto_selector as auto_selector

from constants import RobotButtonMap as RBM

from controllers.driver import DriverController
from controllers.commander import CommanderController
from misc import bor_rot, is_sim, borx, bory


class MyRobot(TimedCommandRobot):
    def robotInit(self) -> None:
        # Robot initialization function
        # We use this timer to get an accurate measurement of how long it takes
        # to process the vision data.
        self.vision_timer = Timer()
        # The field object is something we can send to the SmartDashboard
        # and then many tools can use it to display the robot's position
        self.field = Field2d()
        SmartDashboard.putData(self.field)
        pn = SmartDashboard.putNumber

        # Set up the fake vision data for the simulator
        pn('visiontest/fakeX', 3)
        pn('visiontest/fakeY', 4)
        pn('visiontest/fakeRot', 0)

        if True:
            # Disable the joystick warnings; they're annoying
            DriverStation.silenceJoystickConnectionWarning(True)

        # Now we begin creating each 'subsystem' for the robot.
        # These roughly correspond got a piece of hardware or collection
        # of hardware that we consider one component.
        self.driver_joystick = Joystick(RBM.driver_controller)
        self.commander_joystick1 = Joystick(RBM.commander_controller_1)
        self.commander_joystick2 = Joystick(RBM.commander_controller_2)
        self.driver = DriverController(self.driver_joystick)
        self.commander = CommanderController(self.commander_joystick1,
                                             self.commander_joystick2)

        self.gyro = gyro.Gyro()
        self.photoeyes = photoeyes.Photoeyes()
        self.speaker_tracker = speaker_tracker.SpeakerTracker()
        self.note_tracker = note_tracker.NoteTracker()

        self.amp = amp.Amp(self.commander, self.photoeyes)
        self.shooter = shooter.Shooter()
        self.intake = intake.Intake(self.commander, self.photoeyes)
        self.swerve = drivetrain.Drivetrain(self.gyro, self.driver,
                                            self.intake)
        self.auto_selector = auto_selector.AutoSelector()

        self.climber = climber.Climber(self.driver, self.commander)
        self.climber.encoder_reset()

        # The LEDs tell us the status of the robot so it (generally) needs
        # to know about every other subsystem.
        self.leds = leds.Leds(
            self.amp, self.intake, self.shooter,
            self.swerve, self.note_tracker,
            self.climber, self.photoeyes
        )
        # This allows us to edit the PID values of a PID controller
        # on the fly via the dashboard.
        self.param_editor = param_editor.ParamEditor(
            self.swerve.defcmd.straight_drive_pid
        )

        self.configure_driver_controls()
        self.configure_commander_controls()

    def vision_auto_track(self):
        # Decide wich thing we're going to track
        cmd = None
        # If I have a note in the shooter I should track the speaker
        if self.photoeyes.get_shooter_loaded() is True:
            cmd = DrivetrainHandler(self.speaker_tracker, self.note_tracker,
                                    TrackerType.SPEAKER)
        # if I have a note in the amp mech I should track the amp
        if self.photoeyes.get_amp_loaded() is True:
            cmd = DrivetrainHandler(self.speaker_tracker, self.note_tracker,
                                    TrackerType.AMP)
        # If I don't have a note I should track a note
        elif self.photoeyes.get_intake_loaded() is False:
            cmd = DrivetrainHandler(self.speaker_tracker, self.note_tracker,
                                    TrackerType.NOTE)
        # If we have a command to run, schedule it
        if cmd is not None:
            cmd.schedule()

    def configure_driver_controls(self):
        fr_button = JoystickButton(self.driver_joystick,
                                   RBM.toggle_field_relative)
        fr_button.onTrue(InstantCommand(self.swerve.toggleFieldRelative))

        slow_button = JoystickButton(self.driver_joystick, RBM.slow_mode)
        slow_button.onTrue(
            InstantCommand(self.swerve.slow_mode_on)
        ).onFalse(
            InstantCommand(self.swerve.slow_mode_off)
        )

        flip_button = JoystickButton(self.driver_joystick, RBM.flip_heading)
        flip_button.onTrue(InstantCommand(self.swerve.flipHeading))

        swap_button = JoystickButton(self.driver_joystick, RBM.swap_direction)
        swap_button.onTrue(InstantCommand(self.swerve.swapDirection))

        track_button = JoystickButton(self.driver_joystick, RBM.auto_tracking)
        track_button.onTrue(InstantCommand(self.vision_auto_track))

    def configure_commander_controls(self):
        intake_button = JoystickButton(self.commander_joystick1,
                                       RBM.intake_ready_c1)
        intake_button.whileTrue(IntakeNote(self.intake, self.shooter,
                                           self.gyro, self.photoeyes))

        eject_button = JoystickButton(self.commander_joystick1,
                                      RBM.intake_eject_c1)
        eject_button.whileTrue(EjectNote(self.intake, self.photoeyes))

        amp_load_button = JoystickButton(self.commander_joystick2,
                                         RBM.load_note_amp_c2)
        amp_load_button.onTrue(AmpLoad(self.amp, self.intake, self.photoeyes))

        shooter_load_button = JoystickButton(self.commander_joystick1,
                                             RBM.load_note_shooter_c1)
        shooter_load_button.onTrue(ShooterLoad(self.amp, self.intake,
                                               self.shooter, self.photoeyes))

        amp_set_height_amp = JoystickButton(self.commander_joystick2,
                                            RBM.amp_lift_home_c2)
        amp_set_height_amp.onTrue(SetAmpHeight(self.amp, self.amp.Height.HOME))

        amp_set_height_amp = JoystickButton(self.commander_joystick2,
                                            RBM.amp_lift_amp_c2)
        amp_set_height_amp.onTrue(SetAmpHeight(self.amp, self.amp.Height.AMP))

        amp_set_height_amp = JoystickButton(self.commander_joystick2,
                                            RBM.amp_lift_trap_c2)
        amp_set_height_amp.onTrue(SetAmpHeight(self.amp, self.amp.Height.TRAP))

        amp_override_up = JoystickButton(self.commander_joystick2,
                                         RBM.amp_override_up_c2)
        amp_override_up.whileTrue(SetAmpOverride(self.amp, self.amp.dir_up))

        amp_override_down = JoystickButton(self.commander_joystick2,
                                           RBM.amp_override_down_c2)
        amp_override_down.whileTrue(
            SetAmpOverride(self.amp, self.amp.dir_down)
        )

        shooter_tilt_up = JoystickButton(self.commander_joystick1,
                                         RBM.shooter_override_up_c1)
        shooter_tilt_up.whileTrue(ShooterMove(self.shooter,
                                              self.shooter.dir_up))

        shooter_tilt_down = JoystickButton(self.commander_joystick1,
                                           RBM.shooter_override_down_c1)
        shooter_tilt_down.whileTrue(ShooterMove(self.shooter,
                                                self.shooter.dir_down))

        amp_dump_button = JoystickButton(self.commander_joystick2,
                                         RBM.amp_dump_note_c2)
        amp_dump_button.onTrue(AmpScore(self.amp, self.photoeyes))

        safe_shot_button = JoystickButton(self.commander_joystick1,
                                          RBM.shooter_aim_safe_c1)
        safe_shot_button.onTrue(InstantCommand(self.shooter.safe_shot))

        sub_shot_button = JoystickButton(self.commander_joystick1,
                                         RBM.shooter_aim_sub_c1)
        sub_shot_button.onTrue(InstantCommand(self.shooter.sub_shot))

        shoot_button = JoystickButton(self.commander_joystick2,
                                      RBM.shooter_shoot_c2)
        shoot_button.onTrue(
            ShooterLaunchNote(self.shooter, self.amp, self.commander)
        )
        # shoot_button.onTrue(AutoShooterLaunchNote(self.shooter))

        shooter_spin = JoystickButton(self.commander_joystick2,
                                      RBM.shooter_spin_c2)
        shooter_spin.onTrue(InstantCommand(self.shooter.spin_up))
        shooter_spin.onFalse(InstantCommand(self.shooter.spin_down))

        reset_odo = JoystickButton(self.commander_joystick2,
                                   RBM.reset_odometry_c2)
        reset_odo.onTrue(InstantCommand(self.resetOdometryToCurrentPose))

    # This is a debugging tool. You can use it to tell the swerve drive to
    # trust where the odometry has it and work from there. Might be useful,
    # might not.
    def resetOdometryToCurrentPose(self):
        curr_pose = self.swerve.getPose()
        self.swerve.resetOdometry(curr_pose)

    def robotPeriodic(self) -> None:
        # Rough idea of how to incorporate vision into odometry
        SmartDashboard.putData("Swerve Drivetrain", self.swerve)
        from math import pi
        # Here's our method to pull data from LimeLight's network table
        if is_sim():
            gn = SmartDashboard.getNumber
            fakex = gn('visiontest/fakeX', 0)
            fakey = gn('visiontest/fakeY', 0)
            fakerot = gn('visiontest/fakeRot', 0)
            ll_poses = [Pose2d(fakex, fakey, Rotation2d(radians(fakerot)))]
            certain_within = [(10.0, 10.0, 1/pi)]
            cameralag = 0.0
            computelag = 0.0
        else:
            ll_poses, certain_within, cameralag, computelag = (
                self.get_poses_from_vision()
            )
        # Here we can make a determination to use the vision data or not
        # For example we might only want to accept posees that are already
        # within 1 meter of where we think we are.
        # But after a certain amount of time we might want to accept the
        # pose anyway to correct for drift.
        # We may also need to add a 'fudge factor' to this number
        # to get things performing right. Nothing is set in stone.
        if len(ll_poses) > 0:
            timelag = cameralag + computelag
            for p, cw in zip(ll_poses, certain_within):
                # x = p.X()
                # y = p.Y()
                # rot = p.rotation().degrees()
                # print(f'vision heading: {x}, {y}, {rot}, {ts}')
                ts = self.vision_timer.getFPGATimestamp() - timelag
                xdev, ydev, rotdev = cw
                self.swerve.odometry.addVisionMeasurement(
                    p, ts, (xdev, ydev, rotdev)
                )
        else:
            # print('no vision data')
            pass
        self.field.setRobotPose(self.swerve.getPose())
        pass

    def lock_heading(self, newheading):
        self.swerve.defcmd.desired_heading = newheading

    def autonomousInit(self):
        self.swerve.resetOdometry()
        self.swerve.updateOdometry()
        # TODO: Rethink auton selector. Old one works; but has a lot of code
        auto = self.auton_method()
        auto.schedule()
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.swerve.resetOdometry(Pose2d(0.3, 4, Rotation2d(0)))
        self.swerve.lock_heading()
        self.swerve.fieldRelative = False
        pass

    def teleopPeriodic(self) -> None:
        pass

    # Documentation on JSON return:
    # https://docs.limelightvision.io/docs/docs-limelight/apis/json-dump-specification
    def get_poses_from_vision(self) -> tuple[list[Pose2d],
                                             list[tuple[float, float, float]],
                                             float,
                                             float]:
        t1 = time()
        data = self.speaker_tracker.ll_json_entry.get()
        obj = json.loads(data)
        poses: list[Pose2d] = []
        certain_within: list[tuple[float, float, float]] = []
        camera_lag: float = 0
        # Short circuit any JSON processing if we got back an empty list, which
        # is the default value for the limelight network table entry
        if len(obj) == 0:
            t2 = time()
            diff = t2 - t1
            return poses, certain_within, camera_lag, diff

        result = obj['Results']
        camera_lag = result['tl']
        for fid in result['Fiducial']:
            target_area = fid['ta']
            # Use this, the total area the target is in pixels on the camera
            # picture, to determine how certain we are of the robot's position.

            # start with a very uncertain value unless a condition is met to
            # say otherwise. The format is uncertainty in the x in meters,
            # y in meters, and rotation in degrees
            v = 100.0
            if target_area > 100:
                v = 1.0
            if target_area > 200:
                v = 0.5
            if target_area > 300:
                v = 0.1
            if target_area > 400:
                v = 0.05
            SmartDashboard.putNumber('llacc', v)
            certainty = (v, v, v)

            robot_pose_raw = fid['t6r_fs']
            # It seems like we get a None value from the network table
            # here sometimes and the code crashes if we continue processing it
            # so we'll just skip to the next target if we get a None value.
            if robot_pose_raw is None or len(robot_pose_raw) == 0:
                continue

            offset_x = 8.2296
            offset_y = offset_x / 2

            realx = robot_pose_raw[0] + offset_x
            realy = robot_pose_raw[1] + offset_y
            # Jim Change - the LL t6r_fs gives us "Robot Pose in field space
            # as computed by solvepnp (x,y,z,rx,ry,rz)".
            # We want rotation around the z axis (yaw).
            rot = robot_pose_raw[5]
            pose = Pose2d(realx, realy, Rotation2d(radians(rot)))
            if v < 100:
                poses.append(pose)
                certain_within.append(certainty)
        t2 = time()
        diff = t2 - t1
        return poses, certain_within, camera_lag, diff

    def disabledPeriodic(self) -> None:
        pass

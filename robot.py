# All units of length will be in meters unless otherwise noted

from dataclasses import Field
import json

from time import time
from math import radians
from wpilib import SmartDashboard, Joystick, DriverStation, Timer, Field2d
from commands2 import TimedCommandRobot, SequentialCommandGroup, InstantCommand
from commands2.button import JoystickButton
from wpimath.geometry import Rotation2d, Pose2d
from pathplannerlib.auto import PathPlannerAuto, NamedCommands
from commands.amp_score import AmpScore
from commands.delay import Delay
from commands.auton_commands.auto_shooter_launch_note import (
    AutoShooterLaunchNote
)
from commands.auton_commands.pickup_note import AutoPickupNote
from commands.eject_note import EjectNote

from commands.rotate import Rotate
from commands.set_amp_height import SetAmpHeight
from commands.set_amp_override import SetAmpOverride
from commands.shooter_launch_note import ShooterLaunchNote
from commands.intake_note import IntakeNote

from commands.field_relative_toggle import FieldRelativeToggle

from commands.amp_load import AmpLoad
from commands.shooter_launch_note_test import ShooterLaunchNoteTest
from commands.shooter_load import ShooterLoad
from commands.shooter_move import ShooterMove

import subsystems.amp as amp
import subsystems.climber as climber
import subsystems.gyro as gyro
import subsystems.intake as intake
import subsystems.param_editor as param_editor
import subsystems.shooter as shooter
import subsystems.photoeyes as photoeyes
import subsystems.drivetrain as drivetrain
import subsystems.note_tracker as note_tracker
import subsystems.leds as leds

from constants import RobotButtonMap as RBM

from controllers.driver import DriverController
from controllers.commander import CommanderController

from misc import is_sim, add_timing


class MyRobot(TimedCommandRobot):

    def robotInit(self) -> None:
        self.vision_timer = Timer()
        self.field = Field2d()
        SmartDashboard.putData(self.field)
        pn = SmartDashboard.putNumber
        pn('visiontest/fakeX', 3)
        pn('visiontest/fakeY', 4)
        pn('visiontest/fakeRot', 0)
        """Robot initialization function"""
        if True:
            # Disable the joystick warnings in simulator mode; they're annoying
            DriverStation.silenceJoystickConnectionWarning(True)
        self.driver_joystick = Joystick(RBM.driver_controller)
        self.commander_joystick1 = Joystick(RBM.commander_controller_1)
        self.commander_joystick2 = Joystick(RBM.commander_controller_2)
        self.driver = DriverController(self.driver_joystick)
        self.commander = CommanderController(self.commander_joystick1,
                                             self.commander_joystick2)

        self.gyro = gyro.Gyro()
        self.photoeyes = photoeyes.Photoeyes()

        self.amp = amp.Amp(self.commander, self.photoeyes)
        self.shooter = shooter.Shooter()
        self.note_tracker = note_tracker.NoteTracker()
        self.intake = intake.Intake(self.commander, self.photoeyes)
        self.swerve = drivetrain.Drivetrain(self.gyro, self.driver,
                                            self.note_tracker)
        self.note_tracker = note_tracker.NoteTracker()
        self.climber = climber.Climber(self.driver, self.commander)
        self.leds = leds.Leds(
            self.amp, self.intake, self.shooter,
            self.swerve, self.note_tracker,
            self.climber, self.photoeyes
        )
        self.param_editor = param_editor.ParamEditor(
            self.swerve.defcmd.straight_drive_pid
        )

        sim = is_sim()
        if not sim:
            NamedCommands.registerCommand(
                "PickupNote", AutoPickupNote(self.swerve, self.intake,
                                             self.note_tracker)
            )
            NamedCommands.registerCommand(
                "AutoShoot", AutoShooterLaunchNote(self.shooter)
            )

        self.configure_driver_controls()
        self.configure_commander_controls()

    def configure_driver_controls(self):
        fr_button = JoystickButton(self.driver_joystick,
                                   RBM.toggle_field_relative)
        fr_button.onTrue(InstantCommand(self.swerve.toggleFieldRelative))

        flip_button = JoystickButton(self.driver_joystick, RBM.flip_heading)
        flip_button.onTrue(InstantCommand(self.swerve.flipHeading))

        swap_button = JoystickButton(self.driver_joystick, RBM.swap_direction)
        swap_button.onTrue(InstantCommand(self.swerve.swapDirection))


    def configure_commander_controls(self):
        intake_button = JoystickButton(self.commander_joystick1,
                                       RBM.intake_ready_c1)
        intake_button.whileTrue(IntakeNote(self.intake, self.shooter,
                                           self.gyro, self.photoeyes))

        eject_button = JoystickButton(self.commander_joystick1,
                                      RBM.intake_eject_c1)
        eject_button.onTrue(EjectNote(self.intake, self.photoeyes))

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


        # TODO: These can be made to do something else; intake doesn't move
        amp_override_up = JoystickButton(self.commander_joystick2,
                                         RBM.amp_override_up_c2)
        amp_override_up.whileTrue(SetAmpOverride(self.amp, self.amp.dir_up))

        amp_override_down = JoystickButton(self.commander_joystick2,
                                           RBM.amp_override_down_c2)
        amp_override_down.whileTrue(SetAmpOverride(self.amp, self.amp.dir_down))

        
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
        shoot_button.onTrue(ShooterLaunchNote(self.shooter))
        # shoot_button.onTrue(AutoShooterLaunchNote(self.shooter))

        shooter_spin = JoystickButton(self.commander_joystick2,
                                      RBM.shooter_spin_c2)
        shooter_spin.onTrue(InstantCommand(self.shooter.spin_up))
        shooter_spin.onFalse(InstantCommand(self.shooter.halt))

    def robotPeriodic(self) -> None:
        # Rough idea of how to incorporate vision into odometry
        if self.swerve.vision_stable is True:
            from math import pi
            # Here's our method to pull data from LimeLight's network table
            if is_sim():
                gn = SmartDashboard.getNumber
                fakex = gn('visiontest/fakeX', 0)
                fakey = gn('visiontest/fakeY', 0)
                fakerot = gn('visiontest/fakeRot', 0)
                ll_poses = [Pose2d(fakex, fakey, Rotation2d(radians(fakerot)))]
                certain_within = [(10, 10, 1/pi)]
                cameralag = 0
                computelag = 0
            else:
                ll_poses, certain_within, cameralag, computelag = (
                    self.get_poses_from_limelight()
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
                    x = p.X()
                    y = p.Y()
                    rot = p.rotation().degrees()
                    # self.swerve.odometry.addVisionMeasurement(p, 0, (0.1, 0.1, 0.1))
                    ts = self.vision_timer.getFPGATimestamp() - timelag
                    print(f'vision heading: {x}, {y}, {rot}, {ts}')
                    xdev, ydev, rotdev = cw
                    self.swerve.odometry.addVisionMeasurement(
                        p, ts, (xdev, ydev, rotdev)
                    )
            else:
                # print('no vision data')
                pass
        self.field.setRobotPose(self.swerve.getPose())
        pass

    def auto_station_1(self):
        delaycmd = Delay(5)
        cmd = PathPlannerAuto("LakeCityTwoNote")
        intake_to_shooter = ShooterLoad(self.amp, self.intake, self.shooter,
                                        self.photoeyes)
        shootcmd = AutoShooterLaunchNote(self.shooter,
                                         shooter.tilt_safe, 80)
        cmds = [
            delaycmd,
            cmd,
            intake_to_shooter,
            shootcmd
        ]
        scg = SequentialCommandGroup(cmds)
        return scg

    def auto_station_2(self):
        delaycmd = Delay(1)
        cmd = PathPlannerAuto("LakeCityTwoNoteCenter")
        intake_to_shooter = ShooterLoad(self.amp, self.intake, self.shooter,
                                        self.photoeyes)
        shootcmd = AutoShooterLaunchNote(self.shooter,
                                         shooter.tilt_safe, 80)
        cmds = [
            delaycmd,
            cmd,
            intake_to_shooter,
            shootcmd
        ]
        scg = SequentialCommandGroup(cmds)
        return scg

    def auto_station_3(self):
        delaycmd = Delay(1)
        cmd = PathPlannerAuto("LakeCityTwoNote3")
        intake_to_shooter = ShooterLoad(self.amp, self.intake, self.shooter,
                                        self.photoeyes)
        shootcmd = AutoShooterLaunchNote(self.shooter,
                                         shooter.tilt_safe, 80)
        cmds = [
            delaycmd,
            cmd,
            intake_to_shooter,
            shootcmd
        ]
        scg = SequentialCommandGroup(cmds)
        return scg

    def autonomousInit(self):
        self.swerve.resetOdometry()
        self.swerve.updateOdometry()
        # cmd = Rotate(self.swerve, self.gyro, 0)
        # cmd = DriveToPoint(self.swerve, self.gyro, 3, 0, 0)
        # seek = DriveOverNote(self.note_tracker, self.swerve)
        # followPath = AutoBuilder.followPath(self.testPathToFollow())
        # haltcmd = HaltDrive(self.swerve)
        # rotcmd = Rotate(self.swerve, self.gyro, -180)
        auto = self.auto_station_1()
        auto.schedule()
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.swerve.lock_heading()
        pass

    def teleopPeriodic(self) -> None:
        pass

    # Documentation on JSON return:
    # https://docs.limelightvision.io/docs/docs-limelight/apis/json-dump-specification
    # We are going to be using 't6r_fs' aka "Robot Pose in field space as
    # computed by this fiducial (x,y,z,rx,ry,rz)"
    def get_poses_from_limelight(self):
        t1 = time()
        data = self.swerve.ll_json_entry.get()
        obj = json.loads(data)
        poses = []
        certain_within = []
        camera_lag = 0
        # Short circuit any JSON processing if we got back an empty list, which
        # is the default value for the limelight network table entry
        if len(obj) == 0:
            t2 = time()
            diff = t2 - t1
            return poses, certain_within, camera_lag, diff

        result = obj['Results']
        # camera_lag = result['tl']
        for fid in result['Fiducial']:
            target_area = fid['ta']
            # Use this, the total area the target is in pixels on the camera
            # picture, to determine how certain we are of the robot's position.

            # start with a very uncertain value unless a condition is met to
            # say otherwise. The format is undertainty in the x in meters,
            # y in meters, and rotation in degrees
            certainty = (10, 10, 5)
            if target_area > 200:
                certainty = (3, 3, 3)
            if target_area > 300:
                certainty = (2, 2, 2)
            if target_area > 400:
                certainty = (1, 1, 1)

            robot_pose_raw = fid['t6r_fs']
            # TODO: Verify that the rotation is the right value
            offset_x = 8.2296
            offset_y = offset_x / 2

            realx = robot_pose_raw[0] + offset_x
            realy = robot_pose_raw[1] + offset_y
            # Jim Change - the LL t6r_fs gives us "Robot Pose in field space as computed by solvepnp (x,y,z,rx,ry,rz)".  We want rotation around the z axis (yaw).
            rot = robot_pose_raw[5]
            pose = Pose2d(realx, realy, Rotation2d(radians(rot)))
            poses.append(pose)
            certain_within.append(certainty)
        t2 = time()
        diff = t2 - t1
        return poses, certain_within, camera_lag, diff

    # TODO: Heading needs to be added to the return on this
    # and the overal processing could be a lot cleaner.
    def getVisionXY(self):
        data = self.swerve.ll_json_entry.get()
        obj = json.loads(data)
        totalx, totaly = 0, 0
        currx, curry = None, None
        if len(obj) > 0 and 'Results' in obj.keys():
            obj = obj['Results']
            if 'Fiducial' in obj.keys():
                obj = obj['Fiducial']
                targets = len(obj)
                for f in obj:
                    totalx += f['tx']
                    totaly += f['ty']
                if targets > 0:
                    currx = totalx / targets
                    curry = totaly / targets
        return currx, curry, 0

    def disabledPeriodic(self) -> None:
        pass

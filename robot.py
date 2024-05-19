# All units of length will be in meters unless otherwise noted

from dataclasses import Field
import json
import ntcore

from time import time
from math import radians
from wpilib import SmartDashboard, Joystick, DriverStation, Timer, Field2d
from commands2 import ( ParallelCommandGroup, TimedCommandRobot, SequentialCommandGroup,
                        InstantCommand, CommandScheduler,
)
from commands2.button import JoystickButton, CommandGenericHID, NetworkButton
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
from commands.drivetopoint import DriveToPoint

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
import subsystems.auto_selector as auto_selector

from constants import RobotButtonMap as RBM

from controllers.driver import DriverController
from controllers.commander import CommanderController

from misc import bor_rot, is_sim, add_timing, borx, bory, bor_rot


class AxisButton(JoystickButton):

    def __init__(self, joystick: CommandGenericHID, axis):
        # In 2022 it seems we have to override the isPressed in init
        super().__init__(isPressed=lambda: joystick.getRawAxis(axis) > 0.05)



class MyRobot(TimedCommandRobot):
    def robotInit(self) -> None:
        self.vision_timer = Timer()
        self.field = Field2d()
        self.auton_method = self.auto_station_2_4note
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
                                            self.note_tracker, self.intake)
        self.auto_selector = auto_selector.AutoSelector()

        self.climber = climber.Climber(self.driver, self.commander)
        self.climber.encoder_reset()

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
                "AutoShoot", AutoShooterLaunchNote(self.shooter, self.swerve, rpm=80, do_rotation=False)
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

        note_track_button = JoystickButton(self.driver_joystick,
                                           RBM.note_tracking)
        # note_track_button = AxisButton(self.driver_joystick, 3)
        note_track_button.onTrue(
            InstantCommand(self.swerve.defcmd.note_tracking_on)
        )
        note_track_button.onFalse(
            InstantCommand(self.swerve.defcmd.note_tracking_off)
        )

        speaker_track_button = JoystickButton(self.driver_joystick,
                                              RBM.speaker_tracking)
        speaker_track_button.onTrue(
            InstantCommand(self.swerve.defcmd.speaker_tracking_on)
        )
        speaker_track_button.onFalse(
            InstantCommand(self.swerve.defcmd.speaker_tracking_off)
        )

    def configure_commander_controls(self):
        inst = ntcore.NetworkTableInstance.getDefault()
        nt_table = inst.getTable('virtual_commander')
        intake_button = JoystickButton(self.commander_joystick1,
                                       RBM.intake_ready_c1)
        intake_button_nt = NetworkButton(nt_table, 'intake_ready')

        for b in [intake_button, intake_button_nt]:
            b.whileTrue(IntakeNote(self.intake, self.shooter,
                                   self.gyro, self.photoeyes))

        eject_button = JoystickButton(self.commander_joystick1,
                                      RBM.intake_eject_c1)
        eject_button.whileTrue(EjectNote(self.intake, self.photoeyes))

        amp_load_button = JoystickButton(self.commander_joystick2,
                                         RBM.load_note_amp_c2)
        print('doing the nt button binding')
        al = nt_table.getBooleanTopic('amp_load').getEntry(False)
        amp_load_button_nt = NetworkButton(al)
        amp_load_button.onTrue(AmpLoad(self.amp, self.intake, self.photoeyes))
        amp_load_button_nt.onTrue(AmpLoad(self.amp, self.intake,
                                          self.photoeyes))

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
        shoot_button.onTrue(ShooterLaunchNote(self.shooter, self.amp, self.commander))
        # shoot_button.onTrue(AutoShooterLaunchNote(self.shooter))

        shooter_spin = JoystickButton(self.commander_joystick2,
                                      RBM.shooter_spin_c2)
        shooter_spin.onTrue(InstantCommand(self.shooter.spin_up))
        shooter_spin.onFalse(InstantCommand(self.shooter.spin_down))

        reset_odo = JoystickButton(self.commander_joystick2,
                                   RBM.reset_odometry_c2)
        reset_odo.onTrue(InstantCommand(self.resetOdometryToCurrentPose))

    def resetOdometryToCurrentPose(self):
        curr_pose = self.swerve.getPose()
        self.swerve.resetOdometry(curr_pose)

    def get_auton(self, value) -> None:
        if value == 1:
            self.auton_method = self.auto_station_1
        elif value == 2:
            self.auton_method = self.auto_station_2_4note
        elif value == 3:
            self.auton_method = self.auto_station_2_4note_pole_last
        else:
            self.auton_method = self.auto_station_2_4note

    def robotPeriodic(self) -> None:
        # Rough idea of how to incorporate vision into odometry
        SmartDashboard.putData("Swerve Drivetrain", self.swerve)
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
                    # print(f'vision heading: {x}, {y}, {rot}, {ts}')
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
        delaycmd = Delay(1)
        cmd = PathPlannerAuto("LakeCityTwoNote").asProxy()
        intake_to_shooter = ShooterLoad(self.amp, self.intake, self.shooter,
                                        self.photoeyes).asProxy()
        shootcmd = AutoShooterLaunchNote(self.shooter, self.swerve,
                                         .787, 85, do_rotation = True).asProxy()
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
        shootcmd = AutoShooterLaunchNote(self.shooter, self.swerve,
                                         shooter.tilt_safe, 80).asProxy()
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
        shootcmd = AutoShooterLaunchNote(self.shooter, self.swerve,
                                         shooter.tilt_safe, 80).asProxy()
        cmds = [
            delaycmd,
            cmd,
            intake_to_shooter,
            shootcmd
        ]
        scg = SequentialCommandGroup(cmds)
        return scg

    def auto_station_2_4note(self):
        self.swerve.fieldRelative = True
        flip = self.swerve.shouldFlipPath()
        starting_pose = Pose2d(
            borx(1.5, flip),
            bory(5.5, flip),
            radians(bor_rot(180, flip))
        )
        reset_swerve = self.swerve.resetOdometry(starting_pose)
        delaycmd = Delay(0.25)
        shoot_sub = AutoShooterLaunchNote(self.shooter, self.swerve,
                                          shooter.tilt_sub, 80).asProxy()
        sideways_target = (
            borx(2.0, flip),
            bory(3.75, flip),
            bor_rot(1, flip)
        )
        slide_sideways = DriveToPoint(self.swerve, self.gyro, *sideways_target).asProxy()

        # verify_rotate = Rotate(self.swerve, self.gyro, 0)
        lock_note1 = InstantCommand(self.swerve.defcmd.note_tracking_on)
        slow_note1 = InstantCommand(lambda: self.swerve.set_note_intake_speed(1.0))
        pickup_note1 = IntakeNote(self.intake, self.shooter, self.amp, self.photoeyes).asProxy()
        release_note1 = InstantCommand(self.swerve.defcmd.note_tracking_off)
        defspeed = drivetrain.default_note_intake_speed
        fast_note1 = InstantCommand(lambda: self.swerve.set_note_intake_speed(defspeed))

        """
        smashed_into_pole_pose = Pose2d(
            borx(2.7, flip),
            bory(3.75, flip),
            radians(bor_rot(30, flip))
        )
        reset_pole = InstantCommand(lambda: self.swerve.resetOdometry(smashed_into_pole_pose))
        """
        back_target = (
            borx(2.0, flip),
            bory(3.75, flip),
            bor_rot(0, flip)
        )
        slide_back = DriveToPoint(self.swerve, self.gyro, *back_target).asProxy()
        load_shooter1 = ShooterLoad(self.amp, self.intake, self.shooter, self.photoeyes).asProxy()
        load_slide1 = ParallelCommandGroup(
            [slide_back, load_shooter1]
        )

        verify_rotate_speaker = Rotate(self.swerve, self.gyro, bor_rot(130, flip)).asProxy()
        lock_speaker1 = InstantCommand(self.swerve.defcmd.speaker_tracking_on)
        shoot_safe = AutoShooterLaunchNote(self.shooter, self.swerve,
                                         shooter.tilt_safe, 80).asProxy()
        unlock_speaker1 = InstantCommand(self.swerve.defcmd.speaker_tracking_off)

        lock_note2 = InstantCommand(self.swerve.defcmd.note_tracking_on)
        pickup_note2 = IntakeNote(self.intake, self.shooter, self.amp, self.photoeyes)
        release_note2 = InstantCommand(self.swerve.defcmd.note_tracking_off)
        load_shooter2 = ShooterLoad(self.amp, self.intake, self.shooter, self.photoeyes).asProxy()

        verify_rotate_speaker2 = Rotate(self.swerve, self.gyro, bor_rot(180, flip))

        rotate_slide2 = ParallelCommandGroup(
            [verify_rotate_speaker2, load_shooter2]
        )

        lock_speaker2 = InstantCommand(self.swerve.defcmd.speaker_tracking_on)
        shoot_safe_pos2 = AutoShooterLaunchNote(self.shooter, self.swerve,
                                                shooter.tilt_safe, 80).asProxy()
        unlock_speaker2 = InstantCommand(
            self.swerve.defcmd.speaker_tracking_off
        )

        rotate2 = Rotate(self.swerve, self.gyro, bor_rot(80, flip))
        cmds = [
            reset_swerve,
            delaycmd,
            shoot_sub,
            slide_sideways,
            lock_note1,
            slow_note1,
            pickup_note1,
            release_note1,
            fast_note1,
            # reset_pole,
            load_slide1,
            verify_rotate_speaker,
            lock_speaker1,
            shoot_safe,
            unlock_speaker1,
            rotate2,
            lock_note2,
            pickup_note2,
            release_note2,
            rotate_slide2,
            lock_speaker2,
            shoot_safe_pos2,
            unlock_speaker2,
        ]
        scg = SequentialCommandGroup(cmds)
        return scg

    def auto_station_2_4note_pole_last(self):
        self.swerve.fieldRelative = True
        flip = self.swerve.shouldFlipPath()
        starting_pose = Pose2d(
            borx(1.5, flip),
            bory(5.5, flip),
            radians(bor_rot(180, flip))
        )
        reset_swerve = self.swerve.resetOdometry(starting_pose)
        delaycmd = Delay(0.25)
        shoot_sub = AutoShooterLaunchNote(self.shooter, self.swerve,
                                          shooter.tilt_sub, 80).asProxy()
        back_target = (
            borx(2.0, flip),
            bory(5.5, flip),
            bor_rot(1, flip)
        )
        slide_back = DriveToPoint(
            self.swerve, self.gyro, *back_target
        ).asProxy()

        # verify_rotate = Rotate(self.swerve, self.gyro, 0)
        lock_note2 = InstantCommand(self.swerve.defcmd.note_tracking_on)
        pickup_note2 = IntakeNote(
            self.intake, self.shooter, self.amp, self.photoeyes
        )
        release_note2 = InstantCommand(self.swerve.defcmd.note_tracking_off)
        load_shooter2 = ShooterLoad(
            self.amp, self.intake, self.shooter, self.photoeyes
        ).asProxy()
        rotate_shot2 = Rotate(self.swerve, self.gyro, bor_rot(180, flip))
        load_rotate2 = ParallelCommandGroup([rotate_shot2, load_shooter2])
        shoot2 = AutoShooterLaunchNote(self.shooter, self.swerve,
                                       shooter.tilt_safe, 80, do_rotation=True).asProxy()
        rotate_note3 = Rotate(self.swerve, self.gyro, bor_rot(90, flip))
        lock_note3 = InstantCommand(self.swerve.defcmd.note_tracking_on)
        pickup_note3 = IntakeNote(
            self.intake, self.shooter, self.amp, self.photoeyes
        )
        release_note3 = InstantCommand(self.swerve.defcmd.note_tracking_off)
        rotate_shot3 = Rotate(self.swerve, self.gyro, bor_rot(-150, flip))
        shoot3 = AutoShooterLaunchNote(self.shooter, self.swerve,
                                       shooter.tilt_safe, 80, do_rotation=True).asProxy()
        last_note = (
            borx(2.0, flip),
            bory(3.75, flip),
            bor_rot(1, flip)
        )
        slide_last_note = DriveToPoint(
            self.swerve, self.gyro, *last_note
        ).asProxy()

        lock_note4 = InstantCommand(self.swerve.defcmd.note_tracking_on)
        pickup_note4 = IntakeNote(
            self.intake, self.shooter, self.amp, self.photoeyes
        )
        release_note4 = InstantCommand(self.swerve.defcmd.note_tracking_off)
        load_shooter4 = ShooterLoad(
            self.amp, self.intake, self.shooter, self.photoeyes
        ).asProxy()
        rotate_shot4 = Rotate(self.swerve, self.gyro, bor_rot(150, flip))
        load_rotate4 = ParallelCommandGroup([rotate_shot4, load_shooter4])
        shoot4 = AutoShooterLaunchNote(self.shooter, self.swerve,
                                       shooter.tilt_safe, 80, do_rotation=True).asProxy()

        cmds = [
            reset_swerve,
            delaycmd,
            shoot_sub,
            slide_back,
            lock_note2, pickup_note2, release_note2, rotate_shot2, load_shooter2, shoot2,
            rotate_note3, lock_note3, pickup_note3, release_note3,
            rotate_shot3, shoot3,
            slide_last_note,
            lock_note4, pickup_note4, release_note4, load_rotate4, shoot4
        ]
        scg = SequentialCommandGroup(cmds)
        return scg

    def lock_heading(self, newheading):
        self.swerve.defcmd.desired_heading = newheading

    def autonomousInit(self):
        self.swerve.resetOdometry()
        self.swerve.updateOdometry()
        # cmd = Rotate(self.swerve, self.gyro, 0)
        # cmd = DriveToPoint(self.swerve, self.gyro, 3, 0, 0)
        # seek = DriveOverNote(self.note_tracker, self.swerve)
        # followPath = AutoBuilder.followPath(self.testPathToFollow())
        # haltcmd = HaltDrive(self.swerve)
        # rotcmd = Rotate(self.swerve, self.gyro, -180)2

        # Experimental auton, leaves pole note for last
        # auto = self.auto_station_2_4note_pole_last()
        self.auto_selector_value = self.auto_selector.get_auton()
        self.get_auton(self.auto_selector_value)
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
            v = 100
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
            # Jim Change - the LL t6r_fs gives us "Robot Pose in field space as computed by solvepnp (x,y,z,rx,ry,rz)".  We want rotation around the z axis (yaw).
            rot = robot_pose_raw[5]
            pose = Pose2d(realx, realy, Rotation2d(radians(rot)))
            if v < 100:
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

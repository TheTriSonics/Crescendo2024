# All units of length will be in meters unless otherwise noted

import json

from math import radians
from wpilib import SmartDashboard, Joystick, DriverStation
from commands2 import TimedCommandRobot, SequentialCommandGroup, InstantCommand
from commands2.button import JoystickButton
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from pathplannerlib.auto import PathPlannerAuto, NamedCommands
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)
from pathplannerlib.commands import FollowPathHolonomic
from commands.amp_score import AmpScore
from commands.auton_commands.auto_shooter_launch_note import AutoShooterLaunchNote
from commands.auton_commands.pickup_note import AutoPickupNote
from commands.drive_over_note import DriveOverNote
from commands.eject_note import EjectNote
from commands.return_to_home import ReturnToHome

from commands.rotate import Rotate
from commands.haltdrive import HaltDrive
from commands.drivetopoint import DriveToPoint
from commands.drivefordistance import DriveForDistance
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
        self.leds = leds.Leds()
        self.amp = amp.Amp(self.commander, self.photoeyes)

        self.shooter = shooter.Shooter()
        self.note_tracker = note_tracker.NoteTracker()
        self.intake = intake.Intake(self.commander, self.photoeyes)
        self.swerve = drivetrain.Drivetrain(self.gyro, self.driver, self.note_tracker)
        self.note_tracker = note_tracker.NoteTracker()
        self.climber = climber.Climber(self.driver)

        sim = is_sim()
        if not sim:
            NamedCommands.registerCommand("PickupNote", AutoPickupNote(self.swerve, self.intake, self.note_tracker))
            NamedCommands.registerCommand("AutoShoot", AutoShooterLaunchNote(self.shooter))

        self.configure_driver_controls()
        self.configure_commander_controls()

    def configure_driver_controls(self):
        fr_button = JoystickButton(self.driver_joystick, RBM.toggle_field_relative)
        fr_button.onTrue(InstantCommand(self.swerve.toggleFieldRelative))
        pass

    def configure_commander_controls(self):
        intake_button = JoystickButton(self.commander_joystick1, RBM.intake_ready_c1)
        intake_button.whileTrue(IntakeNote(self.intake, self.shooter, self.gyro, self.photoeyes, self.leds))

        eject_button = JoystickButton(self.commander_joystick1, RBM.intake_eject_c1)
        eject_button.onTrue(EjectNote(self.intake, self.photoeyes, self.leds))

        amp_load_button = JoystickButton(self.commander_joystick2, RBM.load_note_amp_c2)
        amp_load_button.onTrue(AmpLoad(self.amp, self.intake, self.photoeyes))

        shooter_load_button = JoystickButton(self.commander_joystick1, RBM.load_note_shooter_c1)
        shooter_load_button.onTrue(ShooterLoad(self.amp, self.intake, self.shooter, self.photoeyes))

        amp_set_height_amp = JoystickButton(self.commander_joystick2, RBM.amp_lift_home_c2)
        amp_set_height_amp.onTrue(SetAmpHeight(self.amp, self.amp.Height.HOME))

        amp_set_height_amp = JoystickButton(self.commander_joystick2, RBM.amp_lift_amp_c2)
        amp_set_height_amp.onTrue(SetAmpHeight(self.amp, self.amp.Height.AMP))

        amp_set_height_amp = JoystickButton(self.commander_joystick2, RBM.amp_lift_trap_c2)
        amp_set_height_amp.onTrue(SetAmpHeight(self.amp, self.amp.Height.TRAP))

        amp_override_up = JoystickButton(self.commander_joystick2, RBM.amp_override_up_c2)
        amp_override_up.whileTrue(SetAmpOverride(self.amp, self.amp.dir_up))

        amp_override_down = JoystickButton(self.commander_joystick2, RBM.amp_override_down_c2)
        amp_override_down.whileTrue(SetAmpOverride(self.amp, self.amp.dir_down))

        shooter_tilt_up = JoystickButton(self.commander_joystick1, RBM.shooter_override_up_c1)
        shooter_tilt_up.whileTrue(ShooterMove(self.shooter, self.shooter.dir_up))

        shooter_tilt_down = JoystickButton(self.commander_joystick1, RBM.shooter_override_down_c1)
        shooter_tilt_down.whileTrue(ShooterMove(self.shooter, self.shooter.dir_down))

        amp_dump_button = JoystickButton(self.commander_joystick2, RBM.amp_dump_note_c2)
        amp_dump_button.onTrue(AmpScore(self.amp, self.photoeyes))

        safe_shot_button = JoystickButton(self.commander_joystick1, RBM.shooter_aim_safe_c1)
        safe_shot_button.onTrue(InstantCommand(self.shooter.safe_shot))

        sub_shot_button = JoystickButton(self.commander_joystick1, RBM.shooter_aim_sub_c1)
        sub_shot_button.onTrue(InstantCommand(self.shooter.sub_shot))

        shoot_button = JoystickButton(self.commander_joystick2, RBM.shooter_shoot_c2)
        shoot_button.onTrue(ShooterLaunchNote(self.shooter))
        # shoot_button.onTrue(AutoShooterLaunchNote(self.shooter))

        shooter_spin = JoystickButton(self.commander_joystick2, RBM.shooter_spin_c2)
        shooter_spin.onTrue(InstantCommand(self.shooter.spin_up))
        shooter_spin.onFalse(InstantCommand(self.shooter.spin_down))
        pass

    def robotPeriodic(self) -> None:
        if DriverStation.isDisabled():
            self.leds.set_connect_status()

        # SmartDashboard.putNumber("Climber Speed", self.driver.get_climber_trigger())

        # Rough idea of how to incorporate vision into odometry
        if self.swerve.vision_stable is True:
            # Here's our method to pull data from LimeLight's network table
            [ll_poses], cameralag, computelag = self.get_poses_from_limelight()
            # TODO:
            # Here we can make a determination to use the vision data or not
            # For example we might only want to accept posees that are already
            # within 1 meter of where we think we are.
            # But after a certain amount of time we might want to accept the
            # pose anyway to correct for drift.
            # We may also need to add a 'fudge factor' to this number
            # to get things performing right. Nothing is set in stone.
            timelag = cameralag + computelag
            for p in ll_poses:
                self.swerve.odometry.setVisionMeasurementStdDevs(
                    # TODO: This is a placeholder for the actual std devs used
                    # in the Kalman filter
                    (0.1, 0.1, 0.1),
                    timelag
                )
                self.swerve.odometry.addVisionMeasurement(p)
        pass

    def autonomousInit(self):
        self.swerve.resetOdometry()
        self.swerve.updateOdometry()
        self.gyro.set_yaw(0)
        cmd = PathPlannerAuto("LakeCityTwoNote")
        # cmd = Rotate(self.swerve, self.gyro, 0)
        # cmd = DriveToPoint(self.swerve, self.gyro, 3, 0, 0)
        # seek = DriveOverNote(self.note_tracker, self.swerve)
        # followPath = AutoBuilder.followPath(self.testPathToFollow())
        # haltcmd = HaltDrive(self.swerve)
        rotcmd = Rotate(self.swerve, self.gyro, -180)
        shootcmd = AutoShooterLaunchNote(self.shooter,
                                         shooter.tilt_sub*.80, 80)
        scg = SequentialCommandGroup([cmd, rotcmd, shootcmd])
        scg.schedule()
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
    @add_timing
    def get_poses_from_limelight(self) -> tuple[list[Pose2d], float]:
        data = self.swerve.ll_json_entry.get()
        obj = json.loads(data)
        poses = []
        tl = None
        # Short circuit any JSON processing if we got back an empty list, which
        # is the default value for the limelight network table entry
        if len(obj) == 0:
            return poses, tl

        for result in obj['Results']:
            tl = result['tl']
            for fid in obj['Fiducial']:
                robot_pose_raw = fid['t6r_fs']
                # TODO: Verify that the rotation is the right value
                pose = Pose2d(robot_pose_raw[0], robot_pose_raw[1],
                              Rotation2d(radians(robot_pose_raw[3])))
                poses.append(pose)
        return poses, tl

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
                pp = json.dumps(obj, indent=4)
                for f in obj:
                    totalx += f['tx']
                    totaly += f['ty']
                if targets > 0:
                    currx = totalx / targets
                    curry = totaly / targets
        return currx, curry, 0

    def disabledPeriodic(self) -> None:
        pass

    '''
    def followPathCommand(self, pathName: str):
        path = PathPlannerPath.fromPathFile(pathName)
        return FollowPathHolonomic(
            path,
            self.swerve.getPose, # Robot pose supplier
            self.swerve.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.swerve.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
                4.5, # Max module speed, in m/s
                0.4, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.swerve.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
    '''

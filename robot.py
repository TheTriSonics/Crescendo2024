# All units of length will be in meters unless otherwise noted

import json

from wpilib import SmartDashboard, Joystick, DriverStation
from commands2 import TimedCommandRobot, SequentialCommandGroup
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)
from pathplannerlib.commands import FollowPathHolonomic

from commands.rotate import Rotate
from commands.haltdrive import HaltDrive
from commands.drivetopoint import DriveToPoint
from commands.drivefordistance import DriveForDistance
from commands.shooter_launch_note import ShooterLaunchNote

import subsystems.gyro as gyro
import subsystems.intake as intake
import subsystems.shooter as shooter
import subsystems.photoeyes as photoeyes
import subsystems.drivetrain as drivetrain
import subsystems.note_tracker as note_tracker
import subsystems.leds as leds

from constants import RobotButtonMap as RBM

from controllers.thrust_driver import DriverController
from controllers.commander import CommanderController

from misc import is_sim


class MyRobot(TimedCommandRobot):

    def robotInit(self) -> None:
        """Robot initialization function"""
        if is_sim():
            # Disable the joystick warnings in simulator mode; they're annoying
            DriverStation.silenceJoystickConnectionWarning(True)
        self.driver = DriverController(
            Joystick(RBM.driver_controller)
        )
        self.commander = CommanderController(
            Joystick(RBM.commander_controller_1),
            Joystick(RBM.commander_controller_2),
        )
        self.gyro = gyro.Gyro()
        self.photoeyes = photoeyes.Photoeyes()
        self.leds = leds.Leds()

        self.shooter = shooter.Shooter()
        self.note_tracker = note_tracker.NoteTracker()
        self.intake = intake.Intake(self.commander, self.photoeyes)
        self.swerve = drivetrain.Drivetrain(self.gyro, self.driver, self.note_tracker)
        self.note_tracker = note_tracker.NoteTracker()

    def robotPeriodic(self) -> None:
        if DriverStation.isDisabled():
            self.leds.set_connect_status()

    def autonomousInit(self):
        # cmd = DriveForDistance(self.swerve, 50)
        # cmd = HaltDrive(self.swerve)
        self.swerve.resetOdometry()
        self.gyro.set_yaw(45)
        cmd = PathPlannerAuto("Goofy")
        # cmd = Rotate(self.swerve, self.gyro, 0)
        haltcmd = HaltDrive(self.swerve)
        scg = SequentialCommandGroup([cmd, haltcmd])
        scg.schedule()
        """
        drive1 = DriveToPoint(self.swerve, self.gyro, 200, 100, 180)
        halt1 = HaltDrive(self.swerve)
        drive2 = DriveToPoint(self.swerve, self.gyro, 0, 0, 0)
        halt2 = HaltDrive(self.swerve)
        scg = commands2.SequentialCommandGroup([drive1, halt1, drive2, halt2])
        scg.schedule()
        """
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        if self.swerve.lockable is True and self.swerve.locked is False:
            lock_cmd = HaltDrive(self.swerve)
            lock_cmd.schedule()

        # Rough idea of how to incorporate vision into odometry
        if self.swerve.vision_stable is True:
            # Here's our method to pull data from LimeLight's network table
            x, y, heading = self.getVisionXY()
            vpose = Pose2d(x, y, Rotation2d(heading))
            # TODO: This is a placeholder for the actual std devs used in
            # the Kalman filter
            self.swerve.odometry.setVisionMeasurementStdDevs((0.1, 0.1, 0.1))
            self.swerve.odometry.addVisionMeasurement(vpose)
            pass

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

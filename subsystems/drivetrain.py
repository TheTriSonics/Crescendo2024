
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from commands2 import Subsystem, Command
from wpilib import SmartDashboard, DriverStation
from wpimath.filter import SlewRateLimiter, LinearFilter
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController
from wpimath.kinematics import (
    SwerveModuleState, SwerveDrive4Kinematics,
    SwerveModulePosition, ChassisSpeeds
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

from controllers.driver import DriverController

from subsystems.gyro import Gyro
from subsystems.intake import Intake
from subsystems.swervemodule import SwerveModule

from constants import RobotPIDConstants as PIDC
from constants import RobotMotorMap as RMM


kMaxSpeed = 4.5  # meters/second
kMaxAngularSpeed = math.pi * 5  # radians/second

# These functions are used so often it is nice to have some shorthand
# for them. With Python we can assign a function to a variable and then
# use that variable like we would the function itself.
pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
gb = SmartDashboard.getBoolean

# A base string for the SmartDashboard keys for the fake sensors we might
# want to use in testing or simulation
sdbase = 'fakesensors/drivetrain'

curr_note_intake_speed = 2
default_note_intake_speed = 2


class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self, gyro: Gyro, driver_controller, intake: Intake) -> None:
        super().__init__()
        # Set these to the right numbers in centimeters whenever
        # the robot changes
        swerve_offset = 27 / 100  # cm converted to meters
        self.frontLeftLocation = Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = Translation2d(-swerve_offset, -swerve_offset)
        self.gyro = gyro
        self.intake = intake
        self.controller = driver_controller

        # Half the motors need to be inverted to run the right direction and
        # half are in brake mode to slow the robot down faster but also not
        # make it come to a complete stop too quickly.
        # UPDATE: Eventually we moved to all of them in brake mode because we
        # wanted faster deceleration.

        # TIP: If a module fails and needs to be run in competition take
        # it out of brake mode.
        self.frontLeft = SwerveModule(
            RMM.front_left_drive,
            RMM.front_left_turn,
            RMM.front_left_turn_encoder,
            inverted=True,
            brake=True,
            name='Front left')
        self.frontRight = SwerveModule(
            RMM.front_right_drive,
            RMM.front_right_turn,
            RMM.front_right_turn_encoder,
            inverted=False,
            brake=True,
            name='Front right')
        self.backLeft = SwerveModule(
            RMM.back_left_drive,
            RMM.back_left_turn,
            RMM.back_left_turn_encoder,
            inverted=True,
            brake=True,
            name='Back left')
        self.backRight = SwerveModule(
            RMM.back_right_drive,
            RMM.back_right_turn,
            RMM.back_right_turn_encoder,
            inverted=False,
            brake=True,
            name='Back right')

        self.modules = [
            self.frontLeft,
            self.frontRight,
            self.backLeft,
            self.backRight,
        ]

        self.fieldRelative = False

        self.kinematics = SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        woc = 0.05
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            Pose2d(),
            (woc, woc, woc),  # wheel std devs for Kalman filters
            (0.9, 0.9, 0.9),  # vision std devs for Kalman filters
        )

        # NOTE: Because this is in the __init__ method odometry will be reset
        # when the drivetrain is created, not when first used.
        # The drivetrain is genereally created on robot init, or when the
        # roboRio comes online and starts the code. If you want to ensure a
        # reset at a later time you can do another reseetOdometry call instead
        # of trying to move this one around.
        self.resetOdometry()

        # Configure the AutoBuilder for PathPlanner last
        AutoBuilder.configureHolonomic(
            # Robot pose supplier
            self.getPose,
            # Method to reset odometry (will be called if your auto has a
            # starting pose)
            self.resetOdometry,
            # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.getSpeeds,
            # Method that will drive the robot given ROBOT RELATIVE
            # ChassisSpeeds
            self.driveRobotRelative,
            HolonomicPathFollowerConfig(
                PIDConstants(6.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5, 0.0, 0.16),  # Rotation PID constants
                kMaxSpeed,  # Max module speed, in m/s.
                # Drive base radius in meters. Distance from robot center to
                # furthest module.
                0.350,
                # Default path replanning config. See the API for options
                ReplanningConfig()
            ),
            # Supplier to control path flipping based on alliance color
            self.shouldFlipPath,
            self  # Reference to this subsystem to set requirements
        )

        self.defcmd = DrivetrainDefaultCommand(self, self.controller, intake)
        self.setDefaultCommand(self.defcmd)

    def lock_heading(self) -> None:
        self.desired_heading = self.get_heading_rotation_2d().degrees()

    def getSpeeds(self) -> ChassisSpeeds:
        cs = self.kinematics.toChassisSpeeds(
            [m.getState() for m in self.modules]
        )
        return cs

    def driveRobotRelative(self, speeds) -> None:
        self.fieldRelative = False
        states = self.kinematics.toSwerveModuleStates(speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, kMaxSpeed)
        for m, s in zip(self.modules, states):
            m.setDesiredState(s)

    def shouldFlipPath(self) -> bool:
        return self.is_red_alliance()

    def is_red_alliance(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def resetOdometry(self, pose: Pose2d = None) -> None:
        if pose is None:
            self.gyro.set_yaw(0)
            defaultPos = (
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
            )
            self.odometry.resetPosition(
                Rotation2d(), defaultPos, Pose2d()
            )
        else:
            self.gyro.set_yaw(pose.rotation().degrees())
            self.odometry.resetPosition(
                pose.rotation(),
                modulePositions=[m.getPosition() for m in self.modules],
                pose=pose,
            )

    def get_heading_rotation_2d(self) -> Rotation2d:
        yaw = self.gyro.get_yaw()
        return Rotation2d(math.radians(yaw))

    def toggleFieldRelative(self):
        self.fieldRelative = not self.fieldRelative

    # Used to expose a method in the default command to the robot as a whole
    def flipHeading(self):
        self.defcmd.flipHeading()

    # Used to expose a method in the default command to the robot as a whole
    def swapDirection(self):
        self.defcmd.swapDirection()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        robot_centric_force: bool = False,
        periodSeconds: float = 0.02,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param periodSeconds: Time
        """

        # used to the robot to be field relative if we're tracking a note
        if self.fieldRelative and not robot_centric_force:
            flip = self.shouldFlipPath()
            if flip:
                xSpeed = -xSpeed
                ySpeed = -ySpeed
            cs = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.get_heading_rotation_2d(),
                )
        else:
            cs = ChassisSpeeds(xSpeed, ySpeed, rot)

        # First we convert the desired chassis speeds, or what we want the
        # robot as a whole to do into individual module states.
        states = self.kinematics.toSwerveModuleStates(cs)
        # Next, if we are asking any module to do too much we have to
        # scale the request back so the robot can actually handle it
        # we call this desaturation.
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, kMaxSpeed)
        # Next we iterate through the modules and states at the same time
        # and set the desired state of each one.
        for m, s in zip(self.modules, states):
            m.setDesiredState(s)

    def lockWheels(self):
        for m in self.modules:
            m.lock()

    def setStates(self,
                  fl: SwerveModuleState, fr: SwerveModuleState,
                  bl: SwerveModuleState, br: SwerveModuleState):
        for m, s in zip(self.modules, (fl, fr, bl, br)):
            m.setDesiredState(s)

    # return the current angle of each wheel in the swerve modules
    def getAngles(self) -> tuple[float, float, float, float]:
        return (m.getState().angle.radians() for m in self.modules)

    def updateOdometry(self) -> None:
        # Updates the field relative position of the robot.
        self.odometry.update(
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def getPose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def periodic(self) -> None:
        self.updateOdometry()
        pose = self.getPose()
        pb("drivetrain/field_relative", self.fieldRelative)
        pn('drivetrain/odometry/pose_x', pose.X())
        pn('drivetrain/odometry/pose_y', pose.Y())
        pn('drivetrain/odometry/pose_rotation', pose.rotation().degrees())


class DrivetrainDefaultCommand(Command):
    """
    Default command for the drivetrain.
    """
    def __init__(self, drivetrain: Drivetrain, controller: DriverController,
                 gyro: Gyro, intake: Intake) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.controller = controller
        self.gyro = gyro
        self.intake = intake
        self.note_pid = PIDController(*PIDC.note_tracking_pid)
        self.note_translate_pid = PIDController(*PIDC.note_translate_pid)
        self.speaker_pid = PIDController(*PIDC.speaker_tracking_pid)
        self.straight_drive_pid = PIDController(*PIDC.straight_drive_pid)
        self.straight_drive_pid.setTolerance(2.0)
        # Slew rate limiters to make joystick inputs more gentle
        self.xslew = SlewRateLimiter(5.0)
        self.yslew = SlewRateLimiter(5.0)
        self.rotslew = SlewRateLimiter(2)
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

    # Method that uses a PID loop to adjust the robot's heading to keep
    # it pointed straight when the driver is not commanding a rotation
    # Due to mechanical imperfections in the robot's drive train it is
    # going to usually lean to one side or another.
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

    def slow_mode_on(self):
        self.slow_mode = True

    def slow_mode_off(self):
        self.slow_mode = False

    def get_stick_data(self):
        xSpeed = self._get_x_speed()
        ySpeed = self._get_y_speed()
        rot = self._get_rotation()
        return xSpeed, ySpeed, rot

    def execute(self) -> None:
        xSpeed, ySpeed, rot = self.get_stick_data()
        if self.slow_mode:
            slow_mode_factor = 1/2
            xSpeed *= slow_mode_factor
            ySpeed *= slow_mode_factor
            rot *= slow_mode_factor
        self.drivetrain.drive(xSpeed, ySpeed, rot, robot_centric_force=False)

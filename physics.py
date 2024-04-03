import hal.simulation
from math import tau, pi
from wpilib import (
    Field2d, SmartDashboard, RobotController, DriverStation
)
from pyfrc.physics import drivetrains
from robot import MyRobot
from wpilib.simulation import SimDeviceSim
from wpimath.geometry import Pose2d
from wpimath.system.plant import DCMotor
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics
from wpilib.simulation import FlywheelSim, DCMotorSim
from phoenix6.sim import TalonFXSimState, CANcoderSimState, Pigeon2SimState
from phoenix6.hardware import TalonFX
from phoenix6 import unmanaged

pn = SmartDashboard.putNumber
gn = SmartDashboard.getNumber
pb = SmartDashboard.putBoolean
pd = SmartDashboard.putData


class PhysicsEngine:
    bl_drive: TalonFXSimState
    br_drive: TalonFXSimState
    fl_drive: TalonFXSimState
    fr_drive: TalonFXSimState
    blr_drive: TalonFX
    brr_drive: TalonFX
    flr_drive: TalonFX
    frr_drive: TalonFX
    bl_turn: TalonFXSimState
    br_turn: TalonFXSimState
    fl_turn: TalonFXSimState
    fr_turn: TalonFXSimState
    bl_enc: CANcoderSimState
    br_enc: CANcoderSimState
    fl_enc: CANcoderSimState
    fr_enc: CANcoderSimState
    pigeon: Pigeon2SimState

    def __init__(self, physics_controller, robot: MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.pigeon = robot.gyro.gyro.sim_state

        orig_pose = Pose2d(3, 3, 0)
        self.physics_controller.field.setRobotPose(orig_pose)
        self.modules = [robot.swerve.backLeft, robot.swerve.backRight,
                        robot.swerve.frontLeft, robot.swerve.frontRight]
        self.bl_drive, self.br_drive, self.fl_drive, self.fr_drive = [
            m.driveMotor.sim_state for m in self.modules
        ]
        self.blr_drive, self.brr_drive, self.flr_drive, self.frr_drive = [
            m.driveMotor for m in self.modules
        ]
        self.bl_turn, self.br_turn, self.fl_turn, self.fr_turn = [
            m.turningMotor.sim_state for m in self.modules
        ]

        self.bl_enc, self.br_enc, self.fl_enc, self.fr_enc = [
           m.turnEncoder.sim_state for m in self.modules
        ]
        self.drive_motors = [
            self.bl_drive,
            self.br_drive,
            self.fl_drive,
            self.fr_drive,
        ]
        self.turn_motors = [
            self.bl_turn,
            self.br_turn,
            self.fl_turn,
            self.fr_turn,
        ]
        self.turn_encoders = [
            self.bl_enc,
            self.br_enc,
            self.fl_enc,
            self.fr_enc,
        ]
        for m in self.drive_motors + self.turn_encoders:
            m._rotations = 0
        pn('simulator/test_input', 0)

    def update_sim(self, now, tm_diff):
        # We have to run this from the CTRE code to force their devices into
        # the enabled state. The Java and C++ libaries don't have this issue
        # but the workaround here is simple enough
        unmanaged.feed_enable(100)
        bv = RobotController.getBatteryVoltage()
        # Set the input voltage on every CTRE device
        for m in (self.drive_motors +
                  self.turn_motors +
                  self.turn_encoders +
                  [self.pigeon]):
            m.set_supply_voltage(bv)

        for i, dm in enumerate(self.drive_motors):
            dv = dm.motor_voltage
            pn(f'simulator/drive_motor_voltage_{i}', dv)
            volt_to_speed = 0.5  # Fudged number; would have to test real robot
            tick_ratio = 4  # Fudged
            encoder_tick_increment = dv * volt_to_speed * tick_ratio
            dm._rotations += encoder_tick_increment
            dm.set_raw_rotor_position(dm._rotations)
            dm.set_rotor_velocity(encoder_tick_increment / tm_diff)
        for i, (tm, te, sm) in enumerate(
            zip(self.turn_motors, self.turn_encoders, self.modules)
        ):
            tv = tm.motor_voltage
            # pn(f'simulator/turn_motor_voltage_{i}', tv)
            # Instead of a motor encoder we need to adjust the CANcoder
            # Our robot looks for values coming back from -0.5 to 0.5 so that
            # is the range we need to use when setting a value that the robot
            # will scale back into radians/degrees.
            # For now I will use an instant turn
            s = sm.optimized_state_for_sim
            if s is None:
                continue
            desired_angle = s.angle.radians()
            desired_deg = s.angle.degrees()
            v = desired_angle / tau
            pn(f'simulator/desired_angle_{i}', desired_deg)
            pn(f'simulator/desired_val_{i}', v)
            te.set_raw_position(v)

        # Convert our swerve module states into a chassis speed,
        # then we'll get the rotational velocity from that and
        # multiply by our time slice to get the change in angle
        # which we can add to our existing heading.
        cs = self.robot.swerve.getSpeeds()
        if cs is not None:
            from math import degrees
            # Omega is in radians/second
            # angle_change = cs.omega * tm_diff
            # self.pigeon.add_yaw(angle_change)
            # if abs(angle_change) > 0.0001:
            #     print(f'angle_change: {angle_change}')
            dh = self.robot.swerve.defcmd.desired_heading
            if dh is not None:
                self.pigeon.set_raw_yaw(dh)
            else:
                self.pigeon.set_raw_yaw(0)

        """
        print(voltages)
        cs = drivetrains.four_motor_swerve_drivetrain(
            *voltages,
            *angles,
            2, 2,  # Robot dimensions in feet
            0.1,  # Speed in feet per second
            deadzone=None,
        )
        self.physics_controller.drive(cs, tm_diff)
        newpose = self.physics_controller.get_pose()
        self.robot.swerve.resetOdometry(newpose)
        """

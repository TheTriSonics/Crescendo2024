from math import tau, pi, radians, degrees
from random import random
from wpilib.simulation import FlywheelSim, DCMotorSim
from wpimath.system.plant import DCMotor

# Simulated Angle Motor PID Values
SIM_ANGLE_KP = 12.0
SIM_ANGLE_KI = 0.0
SIM_ANGLE_KD = 0.0

# Simulated Drive Motor PID Values
SIM_DRIVE_KP = 0.8
SIM_DRIVE_KI = 0.0
SIM_DRIVE_KD = 0.0

# Simulated Drive Motor Characterization Values
SIM_DRIVE_KS = 0.116970
SIM_DRIVE_KV = 0.133240
SIM_DRIVE_KA = 0.0

SWERVE_GEAR_RATIO = 7.131


class SwerveModuleIOSim:
    def __init__(self):
        self.drive_sim = FlywheelSim(
            DCMotor.falcon500(1), SWERVE_GEAR_RATIO, 0.025
        )
        self.turn_sim = FlywheelSim(
            DCMotor.falcon500(1), SWERVE_GEAR_RATIO, 0.004096955
        )

        turn_relative_position_rad = 0.0
        turn_absolute_position_rad = random() * tau
        drive_applied_volts = 0.0
        turn_applied_volts = 0.0
        is_drive_open_loop = True
        drive_setpoint_mps = 0.0
        angle_setpoint_deg = 0.0
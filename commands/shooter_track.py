from math import atan, dist
from commands2 import Command

from subsystems.shooter import Shooter
from subsystems.drivetrain import Drivetrain

import subsystems.shooter as sm


class ShooterTrack(Command):
    def __init__(self, drive: Drivetrain, shooter: Shooter):
        super().__init__()

        self.shooter = shooter
        self.drive = drive
        self.addRequirements(Shooter)

    def _target_xy(self):
        # 13.5 feet in meters is ... 4.1148 meters
        return 0, 4.1148

    def calc_target_heading(self, pose):
        # Return a heading in degrees that the robot should turn to in order to
        # make a shot on the speaker
        tx, ty = self._target_xy()
        posex = pose.getTranslation().getX()
        posey = pose.getTranslation().getY()
        dx, dy = tx - posex, ty - posey
        taninv = atan(dy/dx)
        if posey < ty:
            return 180-taninv
        else:
            return 180+taninv

    def _deg_to_shooter_tilt(self, deg):
        deg_min = 30
        deg_max = 80
        run = sm.tilt_upper_limit - sm.tilt_bottom_limit
        rise = deg_max - deg_min
        m = rise/run
        b = sm.tilt_bottom_limit
        if deg < deg_min:
            retval = sm.tilt_bottom_limit
        elif deg >= deg_min and deg <= deg_max:
            retval = m*deg + b
        elif deg > deg_max:
            retval = sm.tilt_top_limit
        return retval

    def calc_target_tilt(self, pose):
        base = dist(pose.getTranslation(), self._target_xy())
        rise = 2.5  # Height of target is fixed, and robot can't rise up.
        theta = atan(rise/base)
        return theta

    def execute(self):
        pose = self.drive.getPose()
        heading = self.calc_target_heading(pose)
        tilt_deg = self.calc_target_tilt(pose)
        tilt = self._deg_to_shooter_tilt(tilt_deg)
        self.shooter.tilt_target = tilt


    def end(self, interrupted):
        pass

    # Command will run while a button is held, so the command doesn't
    # ever actually finish; it just gets removed from the scheduler when the
    # button is released
    def isFinished(self):
        return False

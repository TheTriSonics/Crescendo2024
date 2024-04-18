from wpilib import SmartDashboard
from commands2 import Subsystem


class AutoSelector(Subsystem):
    class AutoPath():
        ONE = 1,
        TWO = 2,
        THREE = 3

    def __init__(self):
        super().__init__()
        self.auton_method = 2
        SmartDashboard.putNumber('auton/route', 2)

    def get_auton(self) -> int:
        return self.auton_method

    def periodic(self) -> None:
        gn = SmartDashboard.getNumber
        ps = SmartDashboard.putString
        route = gn('auton/route', 2)
        if route == 1:
            self.auton_method = 1
            ps('auto', 'Lake City 2')
        elif route == 2:
            self.auton_method = 2
            ps('auto', 'GVSU Pole 1st')
        elif route == 3:
            self.auton_method = 3
            ps('auto', 'GVSU Pole last')
        else:
            ps('auto', 'GVSU Pole 1st')
            self.auton_method = 2


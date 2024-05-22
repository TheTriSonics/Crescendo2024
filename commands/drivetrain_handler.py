import wpilib
from wpilib.command import Command

class DrivetrainPriority(Command):
    def __init__(self):
        super().__init__("Drivetrain Priority")
        self.requires(drivetrain_subsystem)  # Replace with your actual drivetrain subsystem

    def initialize(self):
        pass

    def execute(self):
        if driver_controller.getBumper(wpilib.GenericHID.Hand.kLeft):
            rotational_speed = note_lockon.get_rotational_speed()  # Replace with your actual code to get rotational speed from note_lockon file
            drivetrain_subsystem.drive(0, 0, rotational_speed)
        else:
            x_speed = driver_controller.getX(wpilib.GenericHID.Hand.kLeft)
            y_speed = driver_controller.getY(wpilib.GenericHID.Hand.kLeft)
            rotational_speed = driver_controller.getX(wpilib.GenericHID.Hand.kRight)
            drivetrain_subsystem.drive(x_speed, y_speed, rotational_speed)

    def isFinished(self):
        return False

    def end(self):
        drivetrain_subsystem.stop()  # Replace with your actual code to stop the drivetrain

    def interrupted(self):
        self.end()

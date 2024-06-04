import wpilib
from commands2 import Command
from subsystems.drivetrain import Drivetrain
from commands.haltdrive import HaltDrive
from controllers.commander import CommanderController
from wpilib import Joystick

class DrivetrainPriority(Command):
    def __init__(self, controller: CommanderController):
        super().__init__("Drivetrain Priority")
        self.addRequirements(Drivetrain)
        self.controller = controller 

    def execute(self):
        if self.controller.getRawButton(4):
            rotational_speed = note_lockon.get_rotational_speed()  # Replace with your actual code to get rotational speed from note_lockon file
            Drivetrain.drive(0, 0, rotational_speed)
        else:
            x_speed = driver_controller.getX(wpilib.GenericHID.Hand.kLeft)
            y_speed = driver_controller.getY(wpilib.GenericHID.Hand.kLeft)
            rotational_speed = driver_controller.getX(wpilib.GenericHID.Hand.kRight)
            Drivetrain.drive(x_speed, y_speed, rotational_speed)

    def isFinished(self):
        return False

    def end(self):
        HaltDrive # double check that this is right

    # def interrupted(self):
    #    self.end()

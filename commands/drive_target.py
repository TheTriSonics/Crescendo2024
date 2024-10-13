from cmath import sqrt
from math import atan2, degrees
from commands2 import Command
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain
from enum import Enum

class DriveTarget(Command):

    def __init__(self, drivetrain: Drivetrain, target: Enum):
        super().__init__()
        self.drivetrain = drivetrain
        self.target = target
        self.red_alliance = self.drivetrain.isRedAlliance()
        if self.red_alliance:
            self.amp_x = 578.77 / 39.37 # inches to meters, ID 5
            self.amp_y = 323 / 39.37 # inches to meters
            self.amp_heading = 270  # degrees
            self.speaker_x = 652.73 / 39.37 # inches to meters, ID 4
            self.speaker_y = 218.42 / 39.37 # inches to meters
            self.loading_x = 57.54 / 39.37 # inches to meters, ID 10
            self.loading_y = 9.68 / 39.37 # inches to meters
            self.loading_heading = 240  # degrees
            self.stage_l_x = 468.69 / 39.37 # inches to meters, ID 11
            self.stage_l_y = 146.19 / 39.37 # inches to meters
            self.stage_l_heading = 120 # degrees
            self.stage_r_x = 468.69 / 39.37 # inches to meters, ID 12
            self.stage_r_y = 177.12 / 39.37 # inches to meters
            self.stage_r_heading = 240 # degrees
            self.stage_c_x = 441.74 / 39.37 # inches to meters, ID 13
            self.stage_c_y = 161.65 / 39.37 # inches to meters
            self.stage_c_heading = 0 # degrees
        else:
            self.amp_x = 72.5 / 39.37 # inches to meters, ID 6
            self.amp_y = 323 / 39.37 # inches to meters
            self.amp_heading = 270  # degrees
            self.speaker_x = -1.5 / 39.37 # inches to meters, ID 7
            self.speaker_y = 218.42 / 39.37 # inches to meters
            self.loading_x = 593.68 / 39.37 # inches to meters, ID 1
            self.loading_y = 9.68 / 39.37 # inches to meters
            self.loading_heading = 300  # degrees
            self.stage_l_x = 182.73 / 39.37 # inches to meters, ID 15
            self.stage_l_y = 177.1 / 39.37 # inches to meters
            self.stage_l_heading = 300 # degrees
            self.stage_r_x = 182.73 / 39.37 # inches to meters, ID 16
            self.stage_r_y = 146.19 / 39.37 # inches to meters
            self.stage_r_heading = 60 # degrees
            self.stage_c_x = 209.48 / 39.37 # inches to meters, ID 14
            self.stage_c_y = 161.65 / 39.37 # inches to meters
            self.stage_c_heading = 180 # degrees

        # create an enum with options for amp, speaker, and loading
    class Target(Enum):
        AMP = 1
        SPEAKER = 2
        LOADING = 3
        STAGE_L = 4
        STAGE_R = 5
        STAGE_C = 6

    def distanceToAmp(self, currx, curry):
        return sqrt((currx - self.amp_x) ** 2 + (curry - self.amp_y) ** 2)
    
    def distanceToSpeaker(self, currx, curry):
        return sqrt((currx - self.speaker_x) ** 2 + (curry - self.speaker_y) ** 2)
    
    def distanceToLoading(self, currx, curry):
        return sqrt((currx - self.loading_x) ** 2 + (curry - self.loading_y) ** 2)
    
    def distanceToStageL(self, currx, curry):
        return sqrt((currx - self.stage_l_x) ** 2 + (curry - self.stage_l_y) ** 2)
    
    def distanceToStageR(self, currx, curry):
        return sqrt((currx - self.stage_r_x) ** 2 + (curry - self.stage_r_y) ** 2)
    
    def distanceToStageC(self, currx, curry):
        return sqrt((currx - self.stage_c_x) ** 2 + (curry - self.stage_c_y) ** 2)
        
       
    def execute(self):
        pose = self.drivetrain.getPose()
        currx = pose.X()
        curry = pose.Y()
        X = 0
        Y = 0
        heading = 0
        distance = 0

        match self.target:
            case self.Target.AMP:
                X = self.amp_x
                Y = self.amp_y
                heading = self.amp_heading
                distance = self.distanceToAmp(currx, curry)
            case self.Target.SPEAKER:
                X = self.speaker_x
                Y = self.speaker_y
                deltaY = self.speaker_y - curry
                deltaX = self.speaker_x - currx
                heading = degrees(atan2(deltaY, deltaX))
                distance = self.distanceToSpeaker(currx, curry)
            case self.Target.LOADING:
                X = self.loading_x
                Y = self.loading_y
                heading = self.loading_heading
                distance = self.distanceToLoading(currx, curry)
            case self.Target.STAGE_L:
                X = self.stage_l_x
                Y = self.stage_l_y
                heading = self.stage_l_heading
                distance = self.distanceToStageL(currx, curry)
            case self.Target.STAGE_R:
                X = self.stage_r_x
                Y = self.stage_r_y
                heading = self.stage_r_heading
                distance = self.distanceToStageR(currx, curry)
            case self.Target.STAGE_C:
                X = self.stage_c_x
                Y = self.stage_c_y
                heading = self.stage_c_heading
                distance = self.distanceToStageC(currx, curry)

        return X, Y, heading, distance
        

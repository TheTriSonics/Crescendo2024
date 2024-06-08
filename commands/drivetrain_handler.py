from enum import Enum
from subsystems.speaker_tracker import SpeakerTracker
from subsystems.note_tracker import NoteTracker
from subsystems.drivetrain import DrivetrainDefaultCommand


# We use an enumeration to keep track of what we are tracking
class TrackerType(Enum):
    SPEAKER = 1
    NOTE = 2
    AMP = 3
    STAGE_POS_L = 4
    STAGE_POS_R = 5
    STAGE_POS_C = 6


# Here we define the DrivetrainHandler class, which is a command that
# handles the drivetrain when tracking an object. It extends the
# DrivetrainDefaultCommand class, which is the command that handles the
# driver input for the drivetrain.
# What we do here is read in the stick data but let the tracking system
# override that if it wants to.
class DrivetrainHandler(DrivetrainDefaultCommand):

    def __init__(self, speaker_tracker: SpeakerTracker,
                 note_tracker: NoteTracker, tracker_type: TrackerType):
        super().__init__(self.drivetrain, self.controller, self.gyro,
                         self.intake)
        self.speaker_tracker = speaker_tracker
        self.note_tracker = note_tracker
        self.tracking = tracker_type

    def execute(self):
        # Just like the default command we start by getting the driver input
        xSpeed, ySpeed, rot = self.get_stick_data()

        robot_centric = False

        # We only update the 'rot' variable if we have a valid desired_rotation
        # from the appropriate subsystem. Otherwise the stick value will be
        # used.
        speaker_rot = self.speaker_tracker.desired_rotation
        note_rot = self.note_tracker.desired_rotation
        amp_rot = self.amp_tracker.desired_rotation
        if self.tracking == TrackerType.SPEAKER and speaker_rot is not None:
            rot = speaker_rot
        elif self.tracking == TrackerType.NOTE and note_rot is not None:
            rot = note_rot
            # Cut the xpeed down too
            xSpeed /= 2
            # and force to robot centric
            robot_centric = True
            # Should we also force the ySpeed to 0?
            # ySpeed = 0
        elif self.tracking == TrackerType.AMP and amp_rot is not None:
            rot = amp_rot
        # add more tracking ideas here.

        self.drivetrain.drive(xSpeed, ySpeed, rot,
                              robot_centric_force=robot_centric)

    def isFinished(self):
        return False

from commands2 import Subsystem
from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPipelineResult import PhotonTrackedTarget
from misc import is_sim


class NoteTracker(Subsystem):

    def __init__(self):
        # Set up the photon ?
        self.camera = PhotonCamera("notecam")
        self.yaw = None
        self.pitch = None
        self.closest = None
        self.largest = None
        pass

    def getYawOffset(self):
        if self.largest is None:
            return None
        return self.largest.yaw

    def getPitchOffset(self):
        if self.largest is None:
            return None
        return self.largest.pitch

    def _closer(self, a: PhotonTrackedTarget, b: PhotonTrackedTarget):
        if a is None:
            return b
        if b is None:
            return a

        # Determine which target is closer
        # TODO: Implement this
        return a  # for now just return the first one

    def _larger(self, a: PhotonTrackedTarget, b: PhotonTrackedTarget):
        if a is None:
            return b
        if b is None:
            return a

        # Determine which target is larger
        # TODO: Implement this
        return a  # for now just return the first one

    def periodic(self):
        if is_sim():
            return
        # Store off data on where the note is
        lr = self.camera.getLatestResult()
        pn = SmartDashboard.putNumber
        hasTargets = lr.hasTargets()
        if hasTargets is False:
            self.closest = None
            self.largest = None
            return  # Skip processing

        targets = lr.getTargets()
        for t in targets:
            # Photon vision sorts identified objects by area
            # with the first object being the largest
            self.closest = t
            self.largest = t
            # TODO: Remove break when the above methods are implemented
            break  # Only process the first target
        # pn('photon/note/closest/yaw', self.closest.yaw)
        # pn('photon/note/closest/pitch', self.closest.pitch)
        # pn('photon/note/largest/yaw', self.largest.yaw)
        # pn('photon/note/largest/pitch', self.largest.pitch)

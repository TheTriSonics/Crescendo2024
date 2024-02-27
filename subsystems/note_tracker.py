from commands2 import Subsystem
from wpilib import SmartDashboard

from photonlibpy.photonCamera import PhotonCamera

class NoteTracker(Subsystem):

    def __init__(self):
        # Set up the photon ?
        self.camera = PhotonCamera("notecam")
        self.yaw = None
        self.pitch = None
        pass
    
    def getYawOffset(self):
        return self.yaw
    
    def getPitchOffset(self):
        return self.pitch


    def periodic(self):
        # Store off data on where the note is
        lr = self.camera.getLatestResult()
        pn = SmartDashboard.putNumber
        hasTargets = lr.hasTargets()
        if hasTargets is False:
            self.yaw = None
            self.pitch = None
            return  # Skip processing

        targets = lr.getTargets()
        for t in targets:
            self.yaw = t.yaw
            self.pitch = t.pitch
            break
        pn('photon/note/yaw', self.yaw)
        pn('photon/note/pitch', self.pitch)
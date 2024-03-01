from commands2 import Subsystem
from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera


class PhotonPose(Subsystem):

    def __init__(self, name: str):
        # Set up the photon ?
        self.camera = PhotonCamera(name)
        self.name = name
        self.pose = None
        self.pose_latency = None
        self.pose_updated_ms = None
        pass

    def periodic(self):
        # Store off data on where the note is
        lr = self.camera.getLatestResult()
        pn = SmartDashboard.putNumber
        hasTargets = lr.hasTargets()
        if hasTargets is False:
            return  # Skip processing

        targets = lr.getTargets()
        for t in targets:
            p = t.getBestCameraToTarget()
            # Now translate the position to the robot
        pass

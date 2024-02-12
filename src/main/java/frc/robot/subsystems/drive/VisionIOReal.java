package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionIOReal implements VisionIO {

    private final PhotonCamera cam;

    public VisionIOReal(String cameraName) {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // Update with camera name
        cam = new PhotonCamera(cameraName);
        // Update robotToCam with cam mounting pos
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);



    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        cam.getLatestResult().
    }
}

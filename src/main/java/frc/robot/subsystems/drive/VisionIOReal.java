package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOReal implements VisionIO {

  private final PhotonCamera cam;
  SimCameraProperties cameraProp = new SimCameraProperties();
  VisionSystemSim visionSim = new VisionSystemSim("main");
  /*Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  // The given target model at the given pose
  VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  // Add this vision target to the vision system simulation to make it visible
  visionSim.addVisionTargets(visionTarget);*/

  public VisionIOReal(String cameraName) {
    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Update with camera name
    cam = new PhotonCamera(cameraName);
    // A 640 x 480 camera with a 0 degree diagonal FOV.
    cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(0)); // rotation not updated
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.9309056222667738, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20); // not updated
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35); // not updated
    cameraProp.setLatencyStdDevMs(5); // not updated

    // Update robotToCam with cam mounting pos
    Transform3d robotToCam =
            new Transform3d(
                    new Translation3d(0.5, 0.0, 0.5),
                    new Rotation3d(
                            0, 0,
                            0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    final PhotonPoseEstimator photonPoseEstimator =
            new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    cam,
                    robotToCam);
    visionSim.addAprilTags(aprilTagFieldLayout);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraResult = cam.getLatestResult();
    inputs.latencyMillis = cam.getLatestResult().getLatencyMillis();
    inputs.driverMode = cam.getDriverMode();
    inputs.timestampSeconds = cam.getLatestResult().getTimestampSeconds();
  }
}

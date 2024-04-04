package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

import static frc.robot.Constants.robotToCam;

public class VisionIOSim implements VisionIO {
  VisionSystemSim cameraSystem;
  PhotonCamera camera;
  SimCameraProperties cameraProp = new SimCameraProperties();
  Supplier<Pose2d> pose2dSupplier;
  /*Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  // The given target model at the given pose
  VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  // Add this vision target to the vision system simulation to make it visible
  cameraSystem.addVisionTargets(visionTarget);*/

  public VisionIOSim(String name, Supplier<Pose2d> poseSupplier) {
    pose2dSupplier = poseSupplier;
    AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // A 640 x 480 camera with a 0 degree diagonal FOV.
      cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70)); // rotation not updated
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.9309056222667738, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(26.1); // not updated
    // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(20.663); // not updated
      cameraProp.setLatencyStdDevMs(5.811); // not updated

    //         Update robotToCam with cameraSystem mounting pos
    cameraSystem = new VisionSystemSim("main");
    cameraSystem.addAprilTags(aprilTagFieldLayout);

    camera = new PhotonCamera(name);
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
      cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(true);
      cameraSim.enableDrawWireframe(false);
    cameraSystem.addCamera(cameraSim, robotToCam[0]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    cameraSystem.update(pose2dSupplier.get());
    inputs.cameraResult = camera.getLatestResult();
    inputs.latencyMillis = camera.getLatestResult().getLatencyMillis();
    inputs.driverMode = camera.getDriverMode();
    inputs.timestampSeconds = camera.getLatestResult().getTimestampSeconds();
  }
}

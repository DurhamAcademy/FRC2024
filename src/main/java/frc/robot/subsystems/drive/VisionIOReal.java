package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.SimCameraProperties;

public class VisionIOReal implements VisionIO {

  private final PhotonCamera cam;

  public VisionIOReal(String cameraName) {

    // Update with camera name
    cam = new PhotonCamera(cameraName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraResult = cam.getLatestResult();
    inputs.latencyMillis = cam.getLatestResult().getLatencyMillis();
    inputs.driverMode = cam.getDriverMode();
    inputs.timestampSeconds = cam.getLatestResult().getTimestampSeconds();
    inputs.connected = cam.isConnected();
    inputs.name = cam.getName();
  }
}

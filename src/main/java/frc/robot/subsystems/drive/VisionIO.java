package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {

  String cameraName = "";
  default String getCameraName() {
    return cameraName;
  }
  default void updateInputs(VisionIOInputs inputs) {
  }

  class VisionIOInputs implements LoggableInputs {
    boolean driverMode = false;
    double latencyMillis = 0;
    double timestampSeconds = 0;
    boolean connected = false;
    String name = "";
    PhotonPipelineResult cameraResult = new PhotonPipelineResult();
    private PhotonPipelineResult.APacketSerde aPacketSerde;

    public VisionIOInputs(String name) {
      this.name = name;
    }

    public VisionIOInputs() {}

    public Transform3d[] arrayToPosition(double[] value) {
      var length = ((int) (value.length / 7.0));
      Transform3d[] data = new Transform3d[length + 1];
      for (int i = 0; i < length; i++) {
        var X = value[i * 7];
        var Y = value[i * 7 + 1];
        var Z = value[i * 7 + 2];
        var rotW = value[i * 7 + 3];
        var rotX = value[i * 7 + 4];
        var rotY = value[i * 7 + 5];
        var rotZ = value[i * 7 + 6];
        data[i] =
                new Transform3d(
                        new Translation3d(X, Y, Z), new Rotation3d(new Quaternion(rotW, rotX, rotY, rotZ)));
      }
      return data;
    }

    public double[] positionToArray(Transform3d... value) {
      double[] data = new double[value.length * 7];
      for (int i = 0; i < value.length; i++) {
        data[i * 7] = value[i].getX();
        data[i * 7 + 1] = value[i].getY();
        data[i * 7 + 2] = value[i].getZ();
        data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
        data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
        data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
        data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
      }
      return data;
    }

    @Override
    public void toLog(LogTable table) {
      table.put("LatencyMillis", latencyMillis);
      table.put("TimestampSeconds", timestampSeconds);
      table.put("Name", name);

      var packet = new Packet(cameraResult.getPacketSize());
      aPacketSerde = new PhotonPipelineResult.APacketSerde();
      aPacketSerde.pack(packet, cameraResult);
      table.put("CameraResultData", packet.getData());
    }

    @Override
    public void fromLog(LogTable table) {
      //            driverMode = table.getBoolean("DriverMode", driverMode);
      latencyMillis = table.get("LatencyMillis", latencyMillis);
      timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
      name = table.get("Name", name);
      LogTable.LogValue cameraResultData = table.get("CameraResultData");
      if (cameraResultData != null) {
        cameraResultData.getRaw();
        cameraResult = PhotonPipelineResult.serde.unpack(new Packet(cameraResultData.getRaw()));
      } else {
        connected = false;
      }

      cameraResult.setTimestampSeconds(timestampSeconds);
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof VisionIOInputs)) return false;

      VisionIOInputs that = (VisionIOInputs) o;

      if (driverMode != that.driverMode) return false;
      if (Double.compare(timestampSeconds, that.timestampSeconds) != 0) return false;
      return cameraResult.equals(that.cameraResult);
    }

    @Override
    public int hashCode() {
      int result;
      long temp;
      result = (driverMode ? 1 : 0);
      temp = Double.doubleToLongBits(latencyMillis);
      result = 31 * result + (int) (temp ^ (temp >>> 32));
      temp = Double.doubleToLongBits(timestampSeconds);
      result = 31 * result + (int) (temp ^ (temp >>> 32));
      result = 31 * result + cameraResult.hashCode();
      return result;
    }
  }
}

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
  default void updateInputs(VisionIOInputs inputs) {
  }

  class VisionIOInputs implements LoggableInputs {
    boolean driverMode = false;
    double latencyMillis = 0;
    double timestampSeconds = 0;
    PhotonPipelineResult cameraResult = new PhotonPipelineResult();
    private PhotonPipelineResult.APacketSerde aPacketSerde;

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
      //            table.put("DriverMode", driverMode);
      table.put("LatencyMillis", latencyMillis);
      table.put("TimestampSeconds", timestampSeconds);
      //            table.put("HasTargets", hasTargets);
      //            table.put("LEDMode", ((LEDMode == null) ? VisionLEDMode.kDefault :
      // LEDMode).name());
      //            table.put("CameraMatrix", cameraMatrix.isPresent() ?
      // cameraMatrix.get().getData() : new double[0]);
      //            table.put("DistanceCoefficients", distanceCoefficients.isPresent() ?
      // distanceCoefficients.get().getData() : new double[0]);
      //            var bestTargetTable = table.getSubtable("BestTarget");
      //            if (bestTarget != null) {
      //                bestTargetTable.put("yaw", bestTarget.getYaw());
      //                bestTargetTable.put("pitch", bestTarget.getPitch());
      //                bestTargetTable.put("area", bestTarget.getArea());
      //                bestTargetTable.put("skew", bestTarget.getSkew());
      //                bestTargetTable.put("fiducialId", bestTarget.getFiducialId());
      //
      //                bestTargetTable.put("bestCameraToTarget",
      // positionToArray(bestTarget.getBestCameraToTarget()));
      //
      //                bestTargetTable.put("altCamToTarget",
      // positionToArray(bestTarget.getAlternateCameraToTarget()));
      //
      //                bestTargetTable.put("poseAmbiguity", bestTarget.getPoseAmbiguity());
      ////
      //                List<TargetCorner> minAreaRectCorners = bestTarget.getMinAreaRectCorners();
      //                if (minAreaRectCorners != null) {
      //                    var minAreaRectCornersXValues = new double[minAreaRectCorners.size()];
      //                    var minAreaRectCornersYValues = new double[minAreaRectCorners.size()];
      //                    for (int i = 0, minAreaRectCornersSize = minAreaRectCorners.size(); i <
      // minAreaRectCornersSize; i++) {
      //                        TargetCorner minAreaRectCorner = minAreaRectCorners.get(i);
      //                        minAreaRectCornersXValues[i] = minAreaRectCorner.x;
      //                        minAreaRectCornersYValues[i] = minAreaRectCorner.y;
      //                    }
      //                    bestTargetTable.put("minAreaRectCorners_X", minAreaRectCornersXValues);
      //                    bestTargetTable.put("minAreaRectCorners_Y", minAreaRectCornersYValues);
      //                }
      //                List<TargetCorner> detectedCorners = bestTarget.getDetectedCorners();
      //                if (detectedCorners != null) {
      //                    var detectedCornersXValues = new double[detectedCorners.size()];
      //                    var detectedCornersYValues = new double[detectedCorners.size()];
      //                    for (int i = 0, minAreaRectCornersSize = detectedCorners.size(); i <
      // minAreaRectCornersSize; i++) {
      //                        TargetCorner detectedCorner = detectedCorners.get(i);
      //                        detectedCornersXValues[i] = detectedCorner.x;
      //                        detectedCornersYValues[i] = detectedCorner.y;
      //                    }
      //                    bestTargetTable.put("detectedCorners_X", detectedCornersXValues);
      //                    bestTargetTable.put("detectedCorners_Y", detectedCornersYValues);
      //                }
      //            }
      //            var targetsTable = table.getSubtable("Targets");
      //            if (targets != null) {
      //                var allDetectedCornersXValues = new double[targets.size() * 4];
      //                var allDetectedCornersYValues = new double[targets.size() * 4];
      //                for (int i = 0; i < targets.size(); i++) {
      //                    var target = targets.get(i);
      //                    var targetTable = targetsTable.getSubtable("Target " + i);
      //
      //                    targetTable.put("yaw", target.getYaw());
      //                    targetTable.put("pitch", target.getPitch());
      //                    targetTable.put("area", target.getArea());
      //                    targetTable.put("skew", target.getSkew());
      //                    targetTable.put("fiducialId", target.getFiducialId());
      //
      //                    targetTable.put("bestCameraToTarget",
      // positionToArray(target.getBestCameraToTarget()));
      //
      //                    targetTable.put("altCamToTarget",
      // positionToArray(target.getAlternateCameraToTarget()));
      //
      //                    targetTable.put("poseAmbiguity", target.getPoseAmbiguity());
      ////
      //                    List<TargetCorner> minAreaRectCorners = target.getMinAreaRectCorners();
      //                    if (minAreaRectCorners != null) {
      //                        var minAreaRectCornersXValues = new
      // double[minAreaRectCorners.size()];
      //                        var minAreaRectCornersYValues = new
      // double[minAreaRectCorners.size()];
      //                        for (int corner = 0, minAreaRectCornersSize =
      // minAreaRectCorners.size(); corner < minAreaRectCornersSize; corner++) {
      //                            TargetCorner minAreaRectCorner = minAreaRectCorners.get(corner);
      //                            minAreaRectCornersXValues[corner] = minAreaRectCorner.x;
      //                            minAreaRectCornersYValues[corner] = minAreaRectCorner.y;
      //                        }
      //                        targetTable.put("minAreaRectCorners_X", minAreaRectCornersXValues);
      //                        targetTable.put("minAreaRectCorners_Y", minAreaRectCornersYValues);
      //                    }
      //                    List<TargetCorner> detectedCorners = target.getDetectedCorners();
      //                    if (detectedCorners != null) {
      //                        var detectedCornersXValues = new double[detectedCorners.size()];
      //                        var detectedCornersYValues = new double[detectedCorners.size()];
      //                        for (int corner = 0, minAreaRectCornersSize =
      // detectedCorners.size(); corner < minAreaRectCornersSize; corner++) {
      //                            TargetCorner detectedCorner = detectedCorners.get(corner);
      //                            detectedCornersXValues[corner] = detectedCorner.x;
      //                            detectedCornersYValues[corner] = detectedCorner.y;
      //                            allDetectedCornersXValues[(i * 4) + corner] = detectedCorner.x;
      //                            allDetectedCornersYValues[(i * 4) + corner] = detectedCorner.y;
      //                        }
      //                        targetTable.put("detectedCorners_X", detectedCornersXValues);
      //                        targetTable.put("detectedCorners_Y", detectedCornersYValues);
      //                    }
      //                }
      //                targetsTable.put("detectedCorners_X", allDetectedCornersXValues);
      //                targetsTable.put("detectedCorners_Y", allDetectedCornersYValues);
      //            }

      var packet = new Packet(cameraResult.getPacketSize());
      aPacketSerde = new PhotonPipelineResult.APacketSerde();
      aPacketSerde.pack(packet, cameraResult);

      table.put("CameraResultData", packet.getData());
    }

    @Override
    public void fromLog(LogTable table) {
      //            driverMode = table.getBoolean("DriverMode", driverMode);
      latencyMillis = table.getDouble("LatencyMillis", latencyMillis);
      timestampSeconds = table.getDouble("TimestampSeconds", timestampSeconds);
      //            hasTargets = table.getBoolean("HasTargets", hasTargets);
      //            LEDMode = VisionLEDMode.valueOf(table.getString("LEDMode", ""));
      //
      //            /*var matrix = table.getDoubleArray("DistanceCoefficients", new double[0]);
      //            distanceCoefficients = matrix.length != 5 ? Optional.empty() :
      //                    Optional.of(new MatBuilder<>(Nat.N5(), Nat.N1())
      //                            .fill(matrix));
      //            matrix = table.getDoubleArray("CameraMatrix", new double[0]);
      //            var cameraMarix = new Matrix<N3,N3>(Nat.N3(), Nat.N3());
      //            cameraMarix.fill(matrix);
      //            cameraMatrix = matrix.length != 9 ? Optional.empty() :
      //                    Optional.of(cameraMarix);*/
      //            var matrix = table.get("DistanceCoefficients", new double[0]);
      //            distanceCoefficients = matrix.length != 5 ? Optional.empty() : Optional.of(new
      // Matrix<>(Nat.N5(), Nat.N1()));
      //            for (int i = 0; i < distanceCoefficients.get().getNumRows(); i++) {
      //                for (int j = 0; j < distanceCoefficients.get().getNumCols(); j++) {
      //                    distanceCoefficients.get().set(i, j, matrix[]);
      //                }
      //            }
      //            matrix = table.getDoubleArray("CameraMatrix", new double[0]);
      //            var tempCameraMatrix = new Matrix<N3,N3>(Nat.N3(), Nat.N3());
      //            for (int i = 0; i < tempCameraMatrix.getNumRows(); i++) {
      //                for (int j = 0; j < tempCameraMatrix.getNumCols(); j++) {
      //                    tempCameraMatrix.set(i, j, matrix[i]);
      //                }
      //            }
      //            cameraMatrix = matrix.length != 9 ? Optional.empty() :
      //                    Optional.of(cameraMarix);
      //
      //
      //            var bestTargetTable = table.getSubtable("BestTarget");
      //            if (bestTarget != null) {
      //                var yaw = bestTargetTable.getDouble("yaw", 0.0);
      //                var pitch = bestTargetTable.getDouble("pitch", 0.0);
      //                var area = bestTargetTable.getDouble("area", 0.0);
      //                var skew = bestTargetTable.getDouble("skew", 0.0);
      //                int fiducialId = (int) bestTargetTable.getInteger("fiducialId", -1);
      //
      //                var bestCameraToTarget =
      // arrayToPosition(bestTargetTable.getDoubleArray("bestCameraToTarget", new double[0]));
      //
      //                var altCamToTarget =
      // arrayToPosition(bestTargetTable.getDoubleArray("altCamToTarget", new double[0]));
      //
      //                var poseAmbiguity = bestTargetTable.getDouble("poseAmbiguity", 0.0);
      //
      //                var minAreaRectCornersXValues =
      // bestTargetTable.getDoubleArray("minAreaRectCorners_X", new double[0]);
      //                var minAreaRectCornersYValues =
      // bestTargetTable.getDoubleArray("minAreaRectCorners_Y", new double[0]);
      //                var minAreaRectCorners = new ArrayList<TargetCorner>();
      //                for (int i = 0,
      //                     minAreaRectCornersSize = Math.min(minAreaRectCornersXValues.length,
      // minAreaRectCornersYValues.length);
      //                     i < minAreaRectCornersSize;
      //                     i++)
      //                    minAreaRectCorners.add(new TargetCorner(
      //                            minAreaRectCornersXValues[i],
      //                            minAreaRectCornersYValues[i]
      //                    ));
      //
      //
      //                var detectedCornersXValues =
      // bestTargetTable.getDoubleArray("detectedCorners_X", new double[0]);
      //                var detectedCornersYValues =
      // bestTargetTable.getDoubleArray("detectedCorners_Y", new double[0]);
      //                var detectedCornersSize = Math.min(detectedCornersXValues.length,
      // detectedCornersYValues.length);
      //                var detectedCorners = new ArrayList<TargetCorner>(detectedCornersSize);
      //                for (int i = 0;
      //                     i < detectedCornersSize;
      //                     i++)
      //                    detectedCorners.add(new TargetCorner(
      //                            detectedCornersXValues[i],
      //                            detectedCornersYValues[i]
      //                    ));
      //
      //                bestTarget = new PhotonTrackedTarget(yaw,
      //                        pitch,
      //                        area,
      //                        skew,
      //                        fiducialId,
      //                        bestCameraToTarget[0],
      //                        altCamToTarget[0],
      //                        poseAmbiguity,
      //                        minAreaRectCorners,
      //                        detectedCorners);
      //            }
      //            var targetsTable = table.getSubtable("Targets");
      //
      //            var keyList = new ArrayList<>(List.copyOf(targetsTable.getAll(true).keySet()));
      //            keyList.sort(String::compareToIgnoreCase);
      //            for (String key : keyList) {
      //                if (!key.contains("Target ")) continue;
      //                if (!key.contains("/fiducialId")) continue;
      //                var newKey = key.substring(key.indexOf("Target "));
      //                newKey = newKey.substring(0, newKey.indexOf("/"));
      //                var index = Integer.parseInt(newKey.split(" ", 2)[1].strip());
      //                var targetTable = targetsTable.getSubtable(newKey);
      //                var yaw = targetTable.getDouble("yaw", 0.0);
      //                var pitch = targetTable.getDouble("pitch", 0.0);
      //                var area = targetTable.getDouble("area", 0.0);
      //                var skew = targetTable.getDouble("skew", 0.0);
      //                int fiducialId = (int) targetTable.getInteger("fiducialId", -1);
      //
      //                var bestCameraToTarget =
      // arrayToPosition(targetTable.getDoubleArray("bestCameraToTarget", new double[0]));
      //
      //                var altCamToTarget =
      // arrayToPosition(targetTable.getDoubleArray("altCamToTarget", new double[0]));
      //
      //                var poseAmbiguity = targetTable.getDouble("poseAmbiguity", 0.0);
      //
      //                var minAreaRectCornersXValues =
      // targetTable.getDoubleArray("minAreaRectCorners_X", new double[0]);
      //                var minAreaRectCornersYValues =
      // targetTable.getDoubleArray("minAreaRectCorners_Y", new double[0]);
      //                var minAreaRectCorners = new ArrayList<TargetCorner>(4);
      //                for (int i = 0,
      //                     minAreaRectCornersSize = Math.min(minAreaRectCornersXValues.length,
      // minAreaRectCornersYValues.length);
      //                     i < minAreaRectCornersSize;
      //                     i++)
      //                    minAreaRectCorners.add(new TargetCorner(
      //                            minAreaRectCornersXValues[i],
      //                            minAreaRectCornersYValues[i]
      //                    ));
      //
      //
      //                var detectedCornersXValues = targetTable.getDoubleArray("detectedCorners_X",
      // new double[0]);
      //                var detectedCornersYValues = targetTable.getDoubleArray("detectedCorners_Y",
      // new double[0]);
      //                var detectedCornersSize = Math.min(detectedCornersXValues.length,
      // detectedCornersYValues.length);
      //                var detectedCorners = new ArrayList<TargetCorner>(detectedCornersSize);
      //                for (int i = 0;
      //                     i < detectedCornersSize;
      //                     i++)
      //                    detectedCorners.add(new TargetCorner(
      //                            detectedCornersXValues[i],
      //                            detectedCornersYValues[i]
      //                    ));
      //
      //                targets.add(index, new PhotonTrackedTarget(yaw,
      //                        pitch,
      //                        area,
      //                        skew,
      //                        fiducialId,
      //                        bestCameraToTarget[0],
      //                        altCamToTarget[0],
      //                        poseAmbiguity,
      //                        minAreaRectCorners,
      //                        detectedCorners));
      //            }

      //            cameraResult = new PhotonPipelineResult(latencyMillis, targets);
      cameraResult = aPacketSerde.unpack(new Packet(table.get("CameraResultData").getRaw()));
      //            var packetData = table.getRaw("CameraResultData", new byte[0]);
      //            var packet = new Packet(packetData);
      //            packet.setData(packetData);
      //            cameraResult.createFromPacket(packet);

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

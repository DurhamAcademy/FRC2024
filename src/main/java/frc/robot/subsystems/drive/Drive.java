// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.robotToCam;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = 4.5;
  private static final double TRACK_WIDTH_X = inchesToMeters(20.75);
  private static final double TRACK_WIDTH_Y = inchesToMeters(20.75);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SwerveModulePosition[] swerveModulePositions;
  private final VisionIO visionIO;
  private final VisionIO.VisionIOInputs visionInputs = new VisionIO.VisionIOInputs();

  //  private final

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveDrivePoseEstimator poseEstimator;
  private Pose2d pose = new Pose2d();
  PIDConstants positionPID = new PIDConstants(1.0, .00);
  PIDConstants rotationPID = new PIDConstants(0.5, .01);
  private Measure<Velocity<Angle>> angularVelocity = RadiansPerSecond.zero();
  private Rotation2d lastGyroRotation = new Rotation2d();

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // from center.
  private final PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCam);
  SwerveDrivePoseEstimator noGyroPoseEstimation;
  Rotation2d noGyroRotation;
  private Pose2d previousPose = new Pose2d();
  double timestampSeconds = 0.0;

  public Drive(
      GyroIO gyroIO,
      VisionIO visionIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.visionIO = visionIO;
    this.gyroIO = gyroIO;

    modules[3] = new Module(flModuleIO, 0);
    modules[2] = new Module(frModuleIO, 1);
    modules[1] = new Module(blModuleIO, 2);
    modules[0] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
            this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
            new HolonomicPathFollowerConfig(positionPID, rotationPID,
                    MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig(true, true)),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
//    Pathfinding.setPathfinder(new LocalADStar());
    //noinspection ToArrayCallWithZeroLengthArrayArgument
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyroInputs.yawPosition,
            new SwerveModulePosition[] {
              modules[0].getPosition(),
              modules[1].getPosition(),
              modules[2].getPosition(),
              modules[3].getPosition()
            },
            new Pose2d(3.0, 5.0, new Rotation2d(3.0)));
    poseEstimator.setVisionMeasurementStdDevs(new Matrix<>(Nat.N3(), Nat.N1(), new double[]{4, 4, 8}));

    swerveModulePositions = new SwerveModulePosition[modules.length];
    noGyroPoseEstimation = null;
    noGyroRotation = null;
  }

  public void periodic() {
    previousPose = pose;


    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    visionIO.updateInputs(visionInputs);
    Logger.processInputs("Drive/Vision", visionInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      //noinspection RedundantArrayCreation
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      //noinspection RedundantArrayCreation
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    updateSwerveModulePositions();
    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }

    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    twist.dtheta *= -1;
    twist.dx *= -1;
    twist.dy *= -1;
    if (gyroInputs.connected) {
      if (noGyroPoseEstimation != null && visionInputs.connected) {
        // todo: make the next line conditional, only update pose if cameras are
        //  online, otherwise don't do it.
        poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose);
        noGyroPoseEstimation = null;
      }
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.

      // update the pose estimator
      pose = poseEstimator.update(gyroInputs.yawPosition, swerveModulePositions);
      angularVelocity = RadiansPerSecond.of(gyroInputs.yawVelocityRadPerSec);

    } else {
      if (noGyroPoseEstimation == null) {
        noGyroRotation = pose.getRotation();
        noGyroPoseEstimation =
            new SwerveDrivePoseEstimator(
                kinematics, pose.getRotation(), swerveModulePositions, pose);
      }

      // Apply the twist (change since last loop cycle) to the current pose
      noGyroRotation =
          pose.rotateBy(noGyroRotation.minus(pose.getRotation())).exp(twist).getRotation();
      pose = noGyroPoseEstimation.update(noGyroRotation, swerveModulePositions);
      angularVelocity = RadiansPerSecond.of(twist.dtheta / .02);
    }

    Logger.recordOutput("pose", pose);
    /*
    _____ R: <-x  y^
    | ^ | G: \|/x ->y
    |___|
    */
//    new UnscentedKalmanFilter<>()
//    Transform2d twistPerDt = getTwistPerDt();
//    poseEstimator.addVisionMeasurement(pose.minus(new Transform2d(gyroInputs.accelY * 0.02 * 0.02 - twistPerDt.getX() * .02, gyroInputs.accelX * 0.02 * 0.02 - twistPerDt.getY() * .02, Rotation2d.fromDegrees(0.0))), new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{0.075, .075, 100.0}));// fixme: add acceleration from gyro

    Optional<EstimatedRobotPose> estPose = photonPoseEstimator.update(visionInputs.cameraResult);
    if (estPose.isPresent() && (timestampSeconds != estPose.get().timestampSeconds)) {
      estPose.ifPresent(
              estimatedRobotPose -> {
                Logger.recordOutput("estRoPose", estimatedRobotPose.estimatedPose);
                Pose2d pose2d = estimatedRobotPose.estimatedPose.toPose2d();
                var shouldUse = true;
                if (
                        shouldUse && ((!DriverStation.isFMSAttached()) ||
                                ((estimatedRobotPose.estimatedPose.getX() <= 16.5) &&
                                        (estimatedRobotPose.estimatedPose.getX() > 0) &&
                                        (estimatedRobotPose.estimatedPose.getZ() <= 1) &&
                                        (estimatedRobotPose.estimatedPose.getZ() > -1) &&
                                        (estimatedRobotPose.estimatedPose.getY() <= 8.2) &&
                                        (estimatedRobotPose.estimatedPose.getY() > 0)))
                ) // only add it if it's less than 1 meter and in the field
                {
                  Matrix<N3, N1> visionMatrix;
                  switch (estimatedRobotPose.targetsUsed.size()) {
                    case 0:
                      visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{16, 16, 32});
                    case 1:

                      var mult = estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity() * 25;
                      if (
                              (pose.getX() <= 16.5) &&
                                      (pose.getX() > 0) &&
                                      (pose.getY() <= 8.2) &&
                                      (pose.getY() > 0)
                      ) mult = mult / 4;
                      visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{8 * mult, 8 * mult, 12 * mult});
                      break;
                    case 2:
                      var avg = 0.0;
                      for (PhotonTrackedTarget photonTrackedTarget : estimatedRobotPose.targetsUsed) {
                        Transform3d camttarg = photonTrackedTarget.getBestCameraToTarget();
                        avg += (Math.pow(camttarg.getX(), 2) +Math.pow(camttarg.getY(), 2) + Math.pow(camttarg.getZ(), 2));
                      }
                      visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{1.75 * avg, 1.75 * avg, 3 * avg});
                    default:
                      visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{0.05, 0.05, 0.2});
                  }
                  poseEstimator.addVisionMeasurement(pose2d, estimatedRobotPose.timestampSeconds, visionMatrix);
//                noGyroPoseEstimation.addVisionMeasurement(pose2d, estimatedRobotPose.timestampSeconds, visionMatrix);
                }
              });
    }
    getTwistPerDt();
    getTagCount();
  }

  private void updateSwerveModulePositions() {
    // populate the list
    for (int i = 0; i < modules.length; i++) swerveModulePositions[i] = modules[i].getPosition();
  }

  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }
  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds flipped = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(flipped, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(Measure<Voltage> voltageMeasure) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(voltageMeasure.in(Volts));
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocityRadPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the average drive velocity in radians/sec. */
  public void populateDriveCharacterizationData(SysIdRoutineLog routineLog) {
    Measure<Velocity<Angle>> driveVelocityAverage = RadiansPerSecond.zero();
    Measure<Angle> drivePositionAverage = Radians.zero();

    for (var module : modules) {
      var motor = routineLog.motor("DriveMotor #" + module.getIndex());
      var angularPosition = module.getCharacterizationDrivePosition();
      var angularVelocity = module.getCharacterizationDriveVelocity();
      motor.angularPosition(angularPosition);
      motor.angularVelocity(angularVelocity);

      drivePositionAverage = drivePositionAverage.plus(angularPosition);
      driveVelocityAverage = driveVelocityAverage.plus(angularVelocity);
    }
    var averageDriveMotor = routineLog.motor("Average DriveMotor");
    averageDriveMotor.angularVelocity(driveVelocityAverage.divide(4.0));
    averageDriveMotor.angularPosition(drivePositionAverage.divide(4.0));
  }

  public void populateTurnCharacterizationData(SysIdRoutineLog routineLog) {
    Measure<Velocity<Angle>> driveVelocityAverage = RadiansPerSecond.zero();
    Measure<Angle> drivePositionAverage = Radians.zero();

    for (var module : modules) {
      var motor = routineLog.motor("TurnMotor #" + module.getIndex());
      var angularPosition = module.getCharacterizationTurnPosition();
      var angularVelocity = module.getCharacterizationTurnVelocity();
      motor.angularPosition(angularPosition);
      motor.angularVelocity(angularVelocity);

      driveVelocityAverage = driveVelocityAverage.plus(angularVelocity);
      drivePositionAverage = drivePositionAverage.plus(angularPosition);
    }
    var averageDriveMotor = routineLog.motor("Average TurnMotor");
    averageDriveMotor.angularVelocity(driveVelocityAverage.divide(4.0));
    averageDriveMotor.angularPosition(drivePositionAverage.divide(4.0));
    getVisionTags();
  }

  /**
   * Returns all currently visable apriltags.
   */
  @AutoLogOutput(key = "Vision/Tags 3D")
  private Pose3d[] getVisionTags() {
    List<PhotonTrackedTarget> targets = visionInputs.cameraResult.getTargets();
    var out = new Pose3d[targets.size()];
    for (int i = 0; i < targets.size(); i++)
      out[i] = new Pose3d(pose).plus(robotToCam.plus(targets.get(i).getBestCameraToTarget()));
    return out;
  }

  @AutoLogOutput(key = "Vision/Tags 2D")
  private Pose2d[] getVision2dTags() {
    List<PhotonTrackedTarget> targets = visionInputs.cameraResult.getTargets();
    var out = new Pose2d[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      var b = robotToCam.plus(targets.get(i).getBestCameraToTarget());
      out[i] = pose.plus(new Transform2d(b.getTranslation().toTranslation2d(), b.getRotation().toRotation2d()));
    }
    return out;
  }

  @AutoLogOutput(key = "Vision/Tag Ambiguities")
  private double[] getVisionTagAmbiguities() {
    List<PhotonTrackedTarget> targets = visionInputs.cameraResult.getTargets();
    var out = new double[targets.size()];
    for (int i = 0, targetsSize = targets.size(); i < targetsSize; i++) {
      PhotonTrackedTarget target = targets.get(i);
      out[i] = target.getPoseAmbiguity();
    }
    return out;
  }

  @AutoLogOutput(key = "Vision/Corners")
  private Translation2d[] getVisionCorners() {
    List<PhotonTrackedTarget> targets = visionInputs.cameraResult.getTargets();
    var out = new Translation2d[targets.size() * 4];
    for (int i = 0; i < targets.size(); i++) {
      var corners = targets.get(i).getDetectedCorners();
      for (int j = 0; j < corners.size(); j++) {
        TargetCorner corner = corners.get(j);
        out[i + j] = new Translation2d(
                corner.x,
                corner.y);
      }
    }
    for (int i = 0; i < out.length; i++) {
      Translation2d translation2d = out[i];
      if (translation2d == null) out[i] = new Translation2d();
    }
    return out;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the amount of tags visible from the vision system
   */
  @AutoLogOutput(key = "Vision/Tag Count")
  private int getTagCount() {
    return visionInputs.cameraResult.getTargets().size();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  public boolean cameraConnected() {
    return visionInputs.connected;
  }

  /**
   * Returns the current odometry pose.
   */
  @AutoLogOutput(key = "Odometry/Pose Per Delta Time")
  public Transform2d getTwistPerDt() {
    return previousPose.minus(pose).div(.02);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
    if (gyroInputs.connected)
      this.poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose);
    else this.noGyroPoseEstimation.resetPosition(noGyroRotation, swerveModulePositions, pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
            new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),//br
            new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),//bl
            new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),//fr
            new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),//fl
    };
  }

  @AutoLogOutput
  public Measure<Velocity<Angle>> getAnglularVelocity() {
    return this.angularVelocity.negate();
  }

  public Module[] shuffleboardMethod(){
    return modules;
  }
}

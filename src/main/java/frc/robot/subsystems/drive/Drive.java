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
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.*;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
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
  private Rotation2d lastGyroRotation = new Rotation2d();

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Update robotToCam with cameraSystem mounting pos
  Transform3d robotToCam =
      new Transform3d(
              new Translation3d(
                      inchesToMeters(-10.18),
                      inchesToMeters(-7.074),
                      inchesToMeters(8.53) - 0.035
              ),
          new Rotation3d(
                  0,
                  degreesToRadians(55 - 90),
                  degreesToRadians(3)
          )
      ); // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  private final PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCam);
  SwerveDrivePoseEstimator noGyroPoseEstimation;
  Rotation2d noGyroRotation;

  public Drive(
      GyroIO gyroIO,
      VisionIO visionIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.visionIO = visionIO;
    this.gyroIO = gyroIO;

    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        pose1 -> {
          try {
            setPose(pose1);
          } catch (GyroConnectionException e) {
            throw new RuntimeException(e);
          }
        },
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
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

    swerveModulePositions = new SwerveModulePosition[modules.length];
    noGyroPoseEstimation = null;
    noGyroRotation = null;
  }

  public void periodic() {
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
    if (gyroInputs.connected) {
      if (noGyroPoseEstimation != null) {
        // todo: make the next line conditional, only update pose if cameras are
        //  online, otherwise don't do it.
        poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose);
        noGyroPoseEstimation = null;
      }
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.

      // update the pose estimator
      pose = poseEstimator.update(gyroInputs.yawPosition, swerveModulePositions);

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
    }

    Logger.recordOutput("pose", pose);

    Optional<EstimatedRobotPose> estPose = photonPoseEstimator.update(visionInputs.cameraResult);
    estPose.ifPresent(
            estimatedRobotPose -> {
              Logger.recordOutput("estRoPose", estimatedRobotPose.estimatedPose);
              poseEstimator.addVisionMeasurement(
                      estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            });
  }

  /*public void updateVisionPosition(double leftDist, double rightDist, Rotation2d rotation) {
    // Rotation2d rotation; // need to set to something
    poseEstimator.update(rotation, swerveModulePositions);
    var res = cameraSystem.getLatestResult();
    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();
      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
      var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
      poseEstimator.addVisionMeasurement(
          camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
    }
  }*/

  private void updateSwerveModulePositions() {
    // populate the list
    for (int i = 0; i < modules.length; i++) swerveModulePositions[i] = modules[i].getPosition();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
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

  /** Returns the current odometry pose. */
  //  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) throws GyroConnectionException {
    this.pose = pose;
    if (gyroInputs.connected)
      this.poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose);
    else this.noGyroPoseEstimation.resetPosition(noGyroRotation, swerveModulePositions, pose);
    //    throw new GyroConnectionException(
    //        "Pose estimator was written to without gyroscope data (idk what"
    //            + " will happen after this because now if the gyroscope comes back online pose may
    // act weird");
  }

  /**
   * An exception that describes any problem that happens due to a disconnect from the gyroscope.
   */
  public static class GyroConnectionException extends Exception {
    /**
     * Constructs a new exception with the specified detail message. The cause is not initialized,
     * and may subsequently be initialized by a call to {@link #initCause}.
     *
     * @param message the detail message. The detail message is saved for later retrieval by the
     *     {@link #getMessage()} method.
     */
    public GyroConnectionException(String message) {
      super(message);
    }

    /**
     * Constructs a new exception with the specified detail message and cause.
     *
     * <p>Note that the detail message associated with {@code cause} is <i>not</i> automatically
     * incorporated in this exception's detail message.
     *
     * @param message the detail message (which is saved for later retrieval by the {@link
     *     #getMessage()} method).
     * @param cause the cause (which is saved for later retrieval by the {@link #getCause()}
     *     method). (A {@code null} value is permitted, and indicates that the cause is nonexistent
     *     or unknown.)
     * @since 1.4
     */
    public GyroConnectionException(String message, Throwable cause) {
      super(message, cause);
    }

    /**
     * Constructs a new exception with the specified detail message, cause, suppression enabled or
     * disabled, and writable stack trace enabled or disabled.
     *
     * @param message the detail message.
     * @param cause the cause. (A {@code null} value is permitted, and indicates that the cause is
     *     nonexistent or unknown.)
     * @param enableSuppression whether or not suppression is enabled or disabled
     * @param writableStackTrace whether or not the stack trace should be writable
     * @since 1.7
     */
    public GyroConnectionException(
        String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
      super(message, cause, enableSuppression, writableStackTrace);
    }
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
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}

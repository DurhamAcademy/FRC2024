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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.robotToCam;

public class Drive extends SubsystemBase {
    private static final double MAX_LINEAR_SPEED = 5.75;
    private static final double TRACK_WIDTH_X = inchesToMeters(20.75);
    private static final double TRACK_WIDTH_Y = inchesToMeters(20.75);
    private static final double DRIVE_BASE_RADIUS =
            Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    private final GyroIO gyroIO;

    private final NoteDetectionIO noteIO;

    private final NoteDetectionIOInputsAutoLogged noteInputs = new NoteDetectionIOInputsAutoLogged();
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private SwerveModulePosition[] swerveModulePositions;
    private final VisionIO[] visionIO;
    private final VisionIO.VisionIOInputs[] visionInputs;

    //  private final

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d pose = new Pose2d();
    PIDConstants positionPID = new PIDConstants(5, 0);//64 //works at 8
    public PIDConstants rotationPID = new PIDConstants(3, 0);//32+16
    private Measure<Velocity<Angle>> angularVelocity = RadiansPerSecond.zero();
    private Rotation2d lastGyroRotation = new Rotation2d();

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // from center.
    private final PhotonPoseEstimator[] photonPoseEstimator;
    SwerveDrivePoseEstimator noGyroPoseEstimation;
    Rotation2d noGyroRotation;
    private Pose2d previousPose = new Pose2d();
    double[] timestampSeconds;

    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    public Drive(
            GyroIO gyroIO,
            VisionIO visionIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            NoteDetectionIO noteIO) {
        this(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO, new VisionIO[]{visionIO}, noteIO);
    }

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            VisionIO[] visionIO,
            NoteDetectionIO noteIO) {
        this.visionIO = visionIO;
        this.noteIO = noteIO;

        visionInputs = new VisionIO.VisionIOInputs[visionIO.length];
        for (int i = 0, visionIOLength = visionIO.length; i < visionIOLength; i++) {
            visionInputs[i] = new VisionIO.VisionIOInputs(visionIO[i].getCameraName()) {
            };
        }
        timestampSeconds = new double[visionIO.length];
        for (int i = 0, visionIOLength = visionIO.length; i < visionIOLength; i++) {
            timestampSeconds[i] = 0.0;
        }

        this.gyroIO = gyroIO;

        modules[3] = new Module(flModuleIO, 0);
        modules[2] = new Module(frModuleIO, 1);
        modules[1] = new Module(blModuleIO, 2);
        modules[0] = new Module(brModuleIO, 3);

        photonPoseEstimator = new PhotonPoseEstimator[visionInputs.length];
        for (int i = 0; i < visionInputs.length; i++) {
            photonPoseEstimator[i] = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCam[i]);
        }

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(positionPID, rotationPID,
                        MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig(true, true), Robot.defaultPeriodSecs),
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
                        new SwerveModulePosition[]{
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
    Transform2d transform = new Transform2d(0,0, Rotation2d.fromRadians(0));

//    Transform2d accelerometerModifiedVelocity = new Transform2d(0,0, Rotation2d.fromRadians(0));

//    LinearSystem system = new LinearSystem<N3, N5, N3>(
//            new Matrix<>()
//    ).;
//    KalmanFilter<N2, N2, N2> xVelocityKF = new KalmanFilter<N4, N6, N4>(Nat.N4(), Nat.N4())

    public void periodic() {
        previousPose = pose;

        if(this.getCurrentCommand() != null) {
            Logger.recordOutput("Commands/Drive", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Commands/Drive", "");
        }

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        noteIO.updateInputs(noteInputs);
        Logger.processInputs("Drive/Note", noteInputs);

        VisionIO vision = null;
        for (int i = 0, visionIOLength = visionIO.length; i < visionIOLength; i++) {
            vision = visionIO[i];
            vision.updateInputs(visionInputs[i]);
        }
        for (int i = 0; i < visionInputs.length; i++) {
            VisionIO.VisionIOInputs visionInput = visionInputs[i];
            Logger.processInputs("Drive/Vision/" + visionIO[i].getCameraName(), visionInput);
        }

        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled())
            for (var module : modules)
                module.stop();
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            //noinspection RedundantArrayCreation
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            //noinspection RedundantArrayCreation
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        updateSwerveModulePositions();
        // Update odometry

        // Read wheel positions and deltas from each module
        swerveModulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                            swerveModulePositions[moduleIndex].distanceMeters
                                    - lastModulePositions[moduleIndex].distanceMeters,
                            swerveModulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = swerveModulePositions[moduleIndex];
        }


        // The twist represents the motion of the robot since the last
        // loop cycle in x, y, and theta based only on the modules,
        // without the gyro. The gyro is always disconnected in simulation.
        var twist = kinematics.toTwist2d(moduleDeltas);
        this.transform = pose.exp(twist).minus(pose);
        twist.dtheta *= -1;
        if (isGyroConnected()) {
            boolean isAnyCameraConnected = false;
            for (VisionIO.VisionIOInputs visionInput : visionInputs) {
                if (visionInput.connected) {
                    isAnyCameraConnected = true;
                    break;
                }
            }
            if (noGyroPoseEstimation != null && isAnyCameraConnected) {
                // todo: make the next line conditional, only update pose if cameras are
                //  online, otherwise don't do it.
                poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose);
                noGyroPoseEstimation = null;
            }
            // If the gyro is connected, replace the theta component of the twist
            // with the change in angle since the last loop cycle.

            // update the pose estimator
            pose = poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, swerveModulePositions);
            angularVelocity = RadiansPerSecond.of(gyroInputs.yawVelocityRadPerSec);
        } else {
            if (noGyroPoseEstimation == null) {
                noGyroRotation = pose.getRotation();
                noGyroPoseEstimation =
                        new SwerveDrivePoseEstimator(
                                kinematics, pose.getRotation(), swerveModulePositions, pose);
            }

            // Apply the twist (change since last loop cycle) to the current pose
            noGyroRotation = noGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
            pose = noGyroPoseEstimation.updateWithTime(Timer.getFPGATimestamp(), noGyroRotation, swerveModulePositions);
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
//    var visionInput = visionInputs[0];
//
        for (int i = 0, visionInputsLength = visionInputs.length; i < visionInputsLength; i++) {
            VisionIO.VisionIOInputs visionInput = visionInputs[i];
            Optional<EstimatedRobotPose> estPose = photonPoseEstimator[i].update(visionInput.cameraResult);
            if (estPose.isPresent() && (timestampSeconds[i] != estPose.get().timestampSeconds)) {
                var estimatedRobotPose = estPose.get();
                Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Estimated Robot Pose", estimatedRobotPose.estimatedPose);
                Pose2d pose2d = estimatedRobotPose.estimatedPose.toPose2d();
                if (
                                    ((!DriverStation.isFMSAttached()) ||
                                            ((estimatedRobotPose.estimatedPose.getX() <= 16.5) &&
                                                    (estimatedRobotPose.estimatedPose.getX() > 0) &&
                                                    (estimatedRobotPose.estimatedPose.getZ() <= 1) &&
                                                    (estimatedRobotPose.estimatedPose.getZ() > -1) && //fixme: remove Z restriction
                                                    (estimatedRobotPose.estimatedPose.getY() <= 8.2) &&
                                                    (estimatedRobotPose.estimatedPose.getY() > 0)))
                ) // only add it if it's less than 1 meter and in the field
                {
                    Matrix<N3, N1> visionMatrix;
                    switch (estimatedRobotPose.targetsUsed.size()) {
                        case 0:
                            visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{16, 16, 32});
                            break;

                        case 1:
                            PhotonTrackedTarget target = estimatedRobotPose.targetsUsed.get(0);
                            var mult = target.getPoseAmbiguity() * 500;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier/Step 0", mult);
                            mult*=mult;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier/Step 1", mult);
                            if (
                                    (pose.getX() <= 16.5) &&
                                            (pose.getX() > 0) &&
                                            (pose.getY() <= 8.2) &&
                                            (pose.getY() > 0)
                            ) mult = mult / 4;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier/Step 2", mult);
                            if (target.getPoseAmbiguity()<0.4)
                                mult*=Math.pow(Math.abs(estimatedRobotPose.estimatedPose.getZ()),2)+0.01;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier/Step 3", mult);
                            if (target.getArea()>0.05)
                                mult/= (target.getArea()*target.getArea())*8*8;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier/Step 4", mult);
                            for (int j = 0; j < visionInputs.length; j++) {
                                if (i==j) break;
                                if (visionInputs[j].cameraResult.getTargets().size()>visionInputs[i].cameraResult.getTargets().size() && visionInputs[j].timestampSeconds != timestampSeconds[j])
                                    mult += 0.5;
                            }
                            mult += 2;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier/Step 5", mult);
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Single Tag Matrix Multiplier", mult);
                            visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{8 * mult, 8 * mult, 12 * mult});
                            break;

                        default:
                            MultiTargetPNPResult multiTagResult = visionInput.cameraResult.getMultiTagResult();
                            double meterErrorEstimation = (multiTagResult.estimatedPose.bestReprojErr / visionInput.cameraResult.getBestTarget().getArea()) * 0.045;
//                            meterErrorEstimation = 1;
                            Logger.recordOutput("Vision/" + visionIO[i].getCameraName() + "/Multi Tag meterErrorEstimation", meterErrorEstimation);
                            visionMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{1.75 * meterErrorEstimation, 5 * meterErrorEstimation, 3 * meterErrorEstimation});
                            break;

                    }
                    if (isGyroConnected())
                        poseEstimator.addVisionMeasurement(pose2d, estimatedRobotPose.timestampSeconds, visionMatrix);
                    else
                        noGyroPoseEstimation.addVisionMeasurement(pose2d, estimatedRobotPose.timestampSeconds, visionMatrix);
                    poseEstTransform();
                }
            }
        }
        for (int i = 0, visionInputsLength = visionInputs.length; i < visionInputsLength; i++) {
            VisionIO.VisionIOInputs visionInput = visionInputs[i];
            timestampSeconds[i] = visionInput.timestampSeconds;
        }
        getTwistPerDt();
        logCameraData();
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
        ChassisSpeeds flipped = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
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

    /**
     * Stops the drive.
     */
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

    /**
     * Runs forwards at the commanded voltage.
     */
    public void runCharacterizationVolts(Measure<Voltage> voltageMeasure) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(voltageMeasure.in(Volts));
        }
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocityRadPerSec();
        }
        return driveVelocityAverage / 4.0;
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
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

    @AutoLogOutput()
    public Transform3d poseEstTransform() {
        if (visionInputs.length >= 2)
            return visionInputs[1].cameraResult.getMultiTagResult().estimatedPose.best
                    .plus(visionInputs[0].cameraResult.getMultiTagResult().estimatedPose.best.inverse());
        else return null;
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

    private void logCameraData() {
        for (int k = 0; k < visionInputs.length; k++) {
            VisionIO.VisionIOInputs visionInput = visionInputs[k];
            // Corners
            List<PhotonTrackedTarget> targets = visionInput.cameraResult.targets;
            var outCorners = new Translation2d[targets.size() * 4];
            for (int i = 0; i < targets.size(); i++) {
                var corners = targets.get(i).getDetectedCorners();
                for (int j = 0; j < corners.size(); j++) {
                    TargetCorner corner = corners.get(j);
                    outCorners[i + j] = new Translation2d(
                            corner.x,
                            corner.y);
                }
            }
            for (int i = 0; i < outCorners.length; i++) {
                Translation2d translation2d = outCorners[i];
                if (translation2d == null) outCorners[i] = new Translation2d();
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Corners", outCorners);

            // PNP Result
            var a = visionInput.cameraResult.getMultiTagResult().estimatedPose;
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/best", a.best);
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/bestReprojErr", a.bestReprojErr);
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/isPresent", a.isPresent);
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/ambiguity", a.ambiguity);

            // Tag Count
            Logger.recordOutput("Vision/" + visionInput.name + "/Tag Count", visionInput.cameraResult.getTargets().size());

            // Camera Offsets
            Logger.recordOutput("Vision/" + visionInput.name + "/Vision Override", new Pose3d(pose).plus(robotToCam[k]));

            var outAmbiguities = new double[targets.size()];
            for (int i = 0, targetsSize = targets.size(); i < targetsSize; i++) {
                PhotonTrackedTarget target = targets.get(i);
                outAmbiguities[i] = target.getPoseAmbiguity();
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Tag Ambiguities", outAmbiguities);

            var outAreas = new double[targets.size()];
            for (int i = 0, targetsSize = targets.size(); i < targetsSize; i++) {
                PhotonTrackedTarget target = targets.get(i);
                outAreas[i] = target.getArea();
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Tag Areas", outAreas);

            // Tags 2D
            var outTags2D = new Pose2d[targets.size()];
            for (int i = 0; i < targets.size(); i++) {
                var b = robotToCam[k].plus(targets.get(i).getBestCameraToTarget());
                outTags2D[i] = pose.plus(new Transform2d(b.getTranslation().toTranslation2d(), b.getRotation().toRotation2d()));
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Tags 2d", outTags2D);

            var outTags3d = new Pose3d[targets.size()];
            for (int i = 0; i < targets.size(); i++)
                outTags3d[i] = new Pose3d(pose).plus(robotToCam[k].plus(targets.get(i).getBestCameraToTarget()));
            Logger.recordOutput("Vision/" + visionInput.name + "/Tags 3d", outTags3d);
        }
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return pose;
    }

    @AutoLogOutput(key = "Vision/Camera Count")
    public int cameraCount() {
        var cameraCount = 0;
        for (VisionIO.VisionIOInputs visionInput : visionInputs) {
            if (visionInput.connected ) cameraCount++;
        }
        return cameraCount;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Pose Per Delta Time")
    public Transform2d getTwistPerDt() {
        return transform.times(.02);
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
        if (isGyroConnected())
            this.poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose);
        else this.noGyroPoseEstimation.resetPosition(noGyroRotation, swerveModulePositions, pose);
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return MAX_LINEAR_SPEED;
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return MAX_ANGULAR_SPEED;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),//br
                new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),//bl
                new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),//fr
                new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),//fl
        };
    }

    @AutoLogOutput(key = "[BAD] Angular Velocity")
    public Measure<Velocity<Angle>> getAnglularVelocity() {
        return this.angularVelocity.negate();
    }

    public Module[] shuffleboardMethod() {
        return modules;
    }

    /**
     *
     * @return Heading to note, null if no note sighted
     */
    public Double getDetectedNote(){
        if(!noteInputs.tv) {
            return null;
        } else {
            return noteInputs.tx;
        }
    }
}

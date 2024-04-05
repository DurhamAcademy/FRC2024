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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.math.geometry.Rotation2d.fromRotations;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static java.lang.Math.PI;

public class DriveCommands {

    private static final double DEADBAND = 0.1;
    private static final double CANCEL_COMMAND_DEADBAND = 0.2;
    private static final TrapezoidProfile.Constraints rotationConstraints =
            new TrapezoidProfile.Constraints(
                    RadiansPerSecond.of(9.315), RadiansPerSecond.per(Second).of(31.943));

    private DriveCommands() {
    }

  /*
  -------------------
  ---- UTILITIES ----
  -------------------
   */

    public static Translation2d getLinearVelocity(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        // Apply deadband
        double linearMagnitude =
                applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calcaulate new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }


//    public static Command ampAlign(Drive drive) {
//        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromChoreoTrajectory("AmpAlign"),);
//    }

  /*
  ------------------
  ---- COMMANDS ----
  ------------------
   */

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
                                          Drive drive,
                                          DoubleSupplier xSupplier,
                                          DoubleSupplier ySupplier,
                                          DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to field relative speeds & send command
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec(),
                                    drive.getRotation().rotateBy(
                                            getAllianceRotation())));
                },
                drive);
    }

    public static class CommandAndReadySupplier {
        private Command command;
        private BooleanSupplier readySupplier;

        private CommandAndReadySupplier(Command command, BooleanSupplier readySupplier) {
            this.command = command;
            this.readySupplier = readySupplier;
        }

        public Command getCommand() {
            return command;
        }

        public BooleanSupplier getReadySupplier() {
            return readySupplier;
        }
    }

    public static Command intakeAlign(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier)
    {

        var command =
                new RunCommand(
                        () -> {
//                            // Calculate new linear velocity
                            Translation2d linearVelocity = getLinearVelocity(xSupplier, ySupplier);

                            Double detectedNote = drive.getDetectedNote();

                            if(detectedNote != null) {
                                // we have a target
                                detectedNote *= -0.1;
                            } else {
                                // fall back to stick control
                                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                                omega = Math.copySign(omega * omega, omega);
                                detectedNote = omega * drive.getMaxAngularSpeedRadPerSec();
                            }

                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                            detectedNote,
                                            drive.getRotation().rotateBy(getAllianceRotation())));
                        }, drive);

        return command;
    }


    public static CommandAndReadySupplier aimAtSpeakerCommand(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {

        final Pose2d[] previousPose = {null};
        ProfiledPIDController rotationController =
                new ProfiledPIDController(drive.rotationPID.kP, 0, drive.rotationPID.kD, rotationConstraints);

        rotationController.enableContinuousInput(-PI, PI);
        var filter = LinearFilter.singlePoleIIR(0.08, 0.02);

        var command =
                new RunCommand(
                        () -> {
                            // Calculate new linear velocity
                            Translation2d linearVelocity = getLinearVelocity(xSupplier, ySupplier);
                            // Get the angle to point at the goal
                            var goalAngle =
                                    ShooterCommands.getSpeakerPos().toPose2d()
                                            .getTranslation()
                                            .minus(drive.getPose().getTranslation())
                                            .getAngle();
                            Transform2d robotVelocity = drive.getTwistPerDt();
                            Pose2d movingWhileShootingTarget;
                            Pose2d targetPose = ShooterCommands.getSpeakerPos().toPose2d();
                            targetPose = targetPose.plus(new Transform2d(0.0, goalAngle.getSin() * 0.0, new Rotation2d()));
                            if (previousPose[0] != null && false) {
                                double distance =
                                        targetPose
                                                .getTranslation()
                                                .getDistance(previousPose[0].getTranslation());
                                if (distance != 0) {
                                    var noteVelocity = 16.5;
                                    movingWhileShootingTarget =
                                            targetPose.plus(
                                                    robotVelocity.times( distance / noteVelocity));
                                } else movingWhileShootingTarget = targetPose;
                            } else movingWhileShootingTarget = ShooterCommands.getSpeakerPos().toPose2d();
                            Logger.recordOutput("speakerAimTargetPose", movingWhileShootingTarget);


                            Double goalAngleVelocity = null;
                            if (previousPose[0] != null && false) {
                                var previousAngle =
                                        movingWhileShootingTarget
                                                .getTranslation()
                                                .minus(previousPose[0].getTranslation())
                                                .getAngle();
                                var currentAngle = goalAngle;
                                goalAngleVelocity =
                                        currentAngle.minus(previousAngle).getRadians()/Robot.defaultPeriodSecs;
                            } else goalAngleVelocity = 0.0;
                            Logger.recordOutput("Aim/goalAngleVelocity", goalAngleVelocity);
                            // calculate how much speed is needed to get there
//                  rotationController.reset(
//                      new TrapezoidProfile.State(
//                          Radians.of(drive.getRotation().getRadians()),
//                          drive.getAnglularVelocity()));
//                            rotationController.setGoal();
                            var value = rotationController.calculate(
                                    angleModulus(drive.getPose().getRotation().getRadians()),
                                    new TrapezoidProfile.State(goalAngle.getRadians(), goalAngleVelocity)
                            );

                            Logger.recordOutput("Aim/Calculated Value", (value));
                            Logger.recordOutput("Aim/Goal Position", rotationController.getGoal().position);
                            Logger.recordOutput("Aim/Goal Velocity", rotationController.getGoal().velocity);
                            Logger.recordOutput("Aim/Setpoint Position Error", rotationController.getPositionError());
                            Logger.recordOutput("Aim/Setpoint Velocity Error", rotationController.getVelocityError());
                            Logger.recordOutput("Aim/Setpoint Velocity", rotationController.getSetpoint().velocity);
                            Logger.recordOutput("Aim/Setpoint Position", rotationController.getSetpoint().position);
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),

                                            (rotationController.getSetpoint().velocity + value),
                                            drive.getRotation().rotateBy(getAllianceRotation())));
                            previousPose[0] = drive.getPose();
                        }, drive)
                        .beforeStarting(
                                () -> rotationController.reset(
                                        angleModulus(drive.getRotation().getRadians()),drive.getAnglularVelocity().in(RadiansPerSecond)
                                ), drive)
                        .until(
                                () -> {
                                    // if the controller is giving a turn input, end the command
                                    // because the driver is trying to take back control
                                    var isGTE = omegaSupplier.getAsDouble() >= CANCEL_COMMAND_DEADBAND;
                                    var isLTE = omegaSupplier.getAsDouble() <= -CANCEL_COMMAND_DEADBAND;
                                    return !RobotState.isAutonomous() && (isLTE || isGTE);
                                    // until the driver moves the stick, and it is not during autonomous
                                });
        return new CommandAndReadySupplier(command, rotationController::atGoal);
    }

//source angle is: 150 against side wall, 120 against speaker wall

    private static Rotation2d getAllianceRotation() {
        return fromRotations((DriverStation.getAlliance().orElse(Blue) == Red) ? 0.5 : 0.0);
    }

    public static CommandAndReadySupplier aimAtSpeakerCommand(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return aimAtSpeakerCommand(drive, xSupplier, ySupplier, () -> 0.0);
    }
}

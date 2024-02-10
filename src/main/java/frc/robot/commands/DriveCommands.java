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

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {

    private static final double DEADBAND = 0.1;
    private static final double CANCEL_COMMAND_DEADBAND = 0.2;
    private static final double DRIVE_ROTATION_P_VALUE = 35.0;
    private static TrapezoidProfile.Constraints rotationConstraints =
            new TrapezoidProfile.Constraints(
                    RadiansPerSecond.of(5), RadiansPerSecond.per(Second).of(5.0));

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
                    double omega = applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity = getLinearVelocity(xSupplier, ySupplier);

                    // Convert to field relative speeds & send command
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec(),
                                    drive.getRotation()));
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

    public static CommandAndReadySupplier aimAtSpeakerCommand(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {

        Pose2d speakerAimTargetPose = new Pose2d(.25, 5.5, new Rotation2d());
        final Pose2d[] previousPose = {null};
        ProfiledPIDController rotationController =
                new ProfiledPIDController(
                        DRIVE_ROTATION_P_VALUE, 0, .0, new TrapezoidProfile.Constraints(0, 0));

        SmartDashboard.putNumber("rotationPidP", DRIVE_ROTATION_P_VALUE);
        SmartDashboard.putNumber("rotationPidI", 0.0);
        SmartDashboard.putNumber("rotationPidD", 1.5);
        SmartDashboard.putNumber("rotationPidMV", 0.0);
        SmartDashboard.putNumber("rotationPidMA", 0.0);
        rotationController.enableContinuousInput(Rotations.toBaseUnits(-.5), Rotations.toBaseUnits(.5));

        var command = new RunCommand(
                () -> {
                    rotationController.setP(
                            SmartDashboard.getNumber("rotationPidP", DRIVE_ROTATION_P_VALUE));
                    rotationController.setI(SmartDashboard.getNumber("rotationPidI", 0.0));
                    rotationController.setD(SmartDashboard.getNumber("rotationPidD", 0.0));

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity = getLinearVelocity(xSupplier, ySupplier);
                    // Get the angle to point at the goal
                    var goalAngle =
                            speakerAimTargetPose
                                    .getTranslation()
                                    .minus(drive.getPose().getTranslation())
                                    .getAngle();
                    Transform2d robotVelocity;
                    Pose2d movingWhileShootingTarget;
                    if (previousPose[0] != null) {
                        robotVelocity = previousPose[0].minus(drive.getPose());

                        double distance = speakerAimTargetPose.getTranslation().getDistance(previousPose[0].getTranslation());
                        if (distance != 0) {
                            movingWhileShootingTarget = speakerAimTargetPose.plus(
                                    robotVelocity.times(0.02).times(16.5 / distance)
                            );
                        } else movingWhileShootingTarget = speakerAimTargetPose;
                    } else movingWhileShootingTarget = speakerAimTargetPose;
                    Logger.recordOutput("speakerAimTargetPose", movingWhileShootingTarget);
                    /*

                    |--------|
                    |-------|
                    |------|
                    |----|
                    |---|
                    |--|
                    |-|*
                    |=>
                     */


                    Measure<Velocity<Angle>> goalAngleVelocity = null;
                    if (previousPose[0] != null) {
                        var previousAngle = movingWhileShootingTarget
                                .getTranslation()
                                .minus(previousPose[0].getTranslation())
                                .getAngle();
                        var currentAngle = goalAngle;
                        goalAngleVelocity = Radians.of(currentAngle.minus(previousAngle).getRadians()).per(Seconds.of(0.02));
                    } else goalAngleVelocity = RadiansPerSecond.zero();

                    // calculate how much speed is needed to get there
                    var angularSpeed =
                            rotationController.calculate(
                                    Radians.toBaseUnits(
                                            drive
                                                    .getRotation()
                                                    .getRadians()), // ensure we are giving the controller the base units
                                    new TrapezoidProfile.State(Radians.toBaseUnits(goalAngle.getRadians()), goalAngleVelocity.baseUnitMagnitude()),
                                    rotationConstraints);
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    angularSpeed,
                                    drive.getRotation()));
                    previousPose[0] = drive.getPose();
                })
                .until(
                        () -> {
                            // if the controller is giving a turn input, end the command
                            // because the driver is trying to take back control
                            var isGTE = omegaSupplier.getAsDouble() >= CANCEL_COMMAND_DEADBAND;
                            var isLTE = omegaSupplier.getAsDouble() <= -CANCEL_COMMAND_DEADBAND;
                            return isLTE || isGTE;
                        });
        return new CommandAndReadySupplier(
                command,
                () -> rotationController.atGoal()
        );
    }

    public static CommandAndReadySupplier aimAtSpeakerCommand(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return aimAtSpeakerCommand(drive, xSupplier, ySupplier, () -> 0.0);
    }
}

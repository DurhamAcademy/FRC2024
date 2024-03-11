package frc.robot.commands;


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static java.lang.Math.atan;


public class ShooterCommands {
    static double shooterAngleAdjustment = 0.0;

    static InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    public static Pose3d getSpeakerPos() {
        return (DriverStation.getAlliance().orElse(Blue).equals(Blue)) ?
                new Pose3d(0.24, 5.50, 2.13, new Rotation3d()) :
                new Pose3d(16.27, 5.50, 2.13, new Rotation3d());
    }
    static Transform3d shooterOffset = new Transform3d(new Translation3d(0.0, 0.239, .669), new Rotation3d());

    private static double getDistance(Pose3d pose3d) {
        return pose3d.toPose2d().getTranslation().getNorm();
    }

    private static void populateITM() { //im making separate methods for this because I am not sure how much adjustments you would have to make
        distanceToAngle.put(0.0, 0.0);
        distanceToAngle.put(1.1, -0.2941);
        distanceToAngle.put(1.7, -0.4172);
        distanceToAngle.put(2.39, -0.2679);
        distanceToAngle.put(3.506, -0.1229);
        distanceToAngle.put(1000.0, 0.0);
        distanceToRPM.put(0.0, 3500.0);
        distanceToRPM.put(0.894, 3500.0);
        distanceToRPM.put(3.506, 5000.0);
        distanceToRPM.put(1000.0, 4000.0);
    }

    public static Command autoAim(Shooter shooter, Drive drive) {
        populateITM();
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return Commands.run(() -> {
            // Calcaulate new linear velocity
            // Get the angle to point at the goal
            var goalAngle =
                    ShooterCommands.getSpeakerPos().toPose2d()
                            .getTranslation()
                            .minus(drive.getPose().getTranslation())
                            .getAngle();
            Pose3d targetPose = ShooterCommands.getSpeakerPos();
            targetPose = targetPose.plus(new Transform3d(0.0, goalAngle.getSin() * 0.5, 0.0, new Rotation3d()));
            Pose3d shooter1 = new Pose3d(drive.getPose()).plus(shooterOffset);
            Pose3d pose3d = targetPose.relativeTo(shooter1);
            double distance = getDistance(pose3d);
            Logger.recordOutput("distanceFromGoal", distance);
            double atan = atan(pose3d.getZ() / distance);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(atan + distanceToAngle.get(distance)));
            shooter.shooterRunVelocity(distanceToRPM.get(distance));
        }, shooter);
    }

    public static Command JustShoot(Shooter shooter) {
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return Commands.run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(.8));
            shooter.shooterRunVelocity(3500);
        }, shooter);
    }

    public static Command shooterIdle(Shooter shooter) {
        return Commands.run(() -> {
            shooter.shooterRunVelocity(0.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.5));
        }, shooter);
    }

    public static Command shooterSetZero(Shooter shooter) {
        return Commands.run(() -> {
            shooter.shooterRunVelocity(0.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0));
        }, shooter);
    }

    public static Command ampShoot(Shooter shooter) {
        return Commands.run(() -> {
            shooter.shooterRunVelocity(500.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(-0.17));
        }, shooter);
    }

    public static Command addToOffsett(){
        shooterAngleAdjustment += 0.017;
        return Commands.runOnce(() -> shooterAngleAdjustment += 0.017);
    }

    public static Command removeFromoOffset(){
        shooterAngleAdjustment -= 0.017;
        return Commands.runOnce(() -> shooterAngleAdjustment += 0.017);
    }

    private static class Result {
        public final Pose3d pose3d;
        public final double distance;

        public Result(Pose3d pose3d, double distance) {
            this.pose3d = pose3d;
            this.distance = distance;
        }
    }
}

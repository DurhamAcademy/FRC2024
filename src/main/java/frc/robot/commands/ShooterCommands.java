package frc.robot.commands;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static java.lang.Math.abs;
import static java.lang.Math.atan;


public class ShooterCommands {
    static double shooterAngleAdjustment = 0.0;

    static InterpolatingTreeMap<Double, Double> distanceToAngle = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    static Transform3d shooterOffset = new Transform3d(new Translation3d(0.0, 0.239, .669), new Rotation3d());

    public static LoggedDashboardBoolean isOverridden = new LoggedDashboardBoolean("Override Hood Angle", false);
    public static LoggedDashboardNumber overrideAngle = new LoggedDashboardNumber("Overridden Hood Angle", 0.0);

    public static double inverseInterpolate(Double startValue, Double endValue, Double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q - startValue;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }

    public static Pose3d getSpeakerPos() {
        return (DriverStation.getAlliance().orElse(Blue).equals(Blue)) ?
                new Pose3d(0.24, 5.50, 2.13, new Rotation3d()) :
                new Pose3d(16.27, 5.50, 2.13, new Rotation3d());
    }

    public static Pose3d getSourcePos() {
        return (DriverStation.getAlliance().orElse(Blue).equals(Blue)) ?
                new Pose3d(15.428, 0.916, 2.13, new Rotation3d()) :
                new Pose3d(1.122, 0.916, 2.13, new Rotation3d());
    }

    private static double getDistance(Pose3d pose3d) {
        return pose3d.toPose2d().getTranslation().getNorm();
    }

    public static void construct() {
        distanceToAngle.clear();
        distanceToAngle.put(0.0, 0.0);
//        distanceToAngle.put(1.1, -0.2941);
//        distanceToAngle.put(1.7, -0.3472);
//        distanceToAngle.put(2.52, -0.1700); //GOOD VALUES
//        distanceToAngle.put(4.235, 0.0150);
        distanceToAngle.put(1000.0, 0.0);

        distanceToRPM.clear();
        distanceToRPM.put(0.0, 3500.0);
        distanceToRPM.put(0.894, 3500.0);
        distanceToRPM.put(3.506, 5000.0);
        distanceToRPM.put(2.52, 3750.0); //GOOD VALUES
        distanceToRPM.put(4.25, 4000.0);
        distanceToRPM.put(1000.0,
                4000.0);
    }

    public static LoggedDashboardBoolean retractAfterShot = new LoggedDashboardBoolean("Aim/Retract After Shooting", true);
    public static LoggedDashboardNumber flywheelSpeed = new LoggedDashboardNumber("Aim/FlywheelSpeed", 3000);

    public static Command autoAim(Shooter shooter, Drive drive, Feeder feeder) {
        construct();
        flywheelSpeed.periodic();
        isOverridden.periodic();
        overrideAngle.periodic();
        retractAfterShot.periodic();
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return run(() -> {
            if (isOverridden.get()) {
                shooter.setTargetShooterAngle(Rotation2d.fromRadians(overrideAngle.get()));
                shooter.overrideHoodAtSetpoint(true);
                shooter.shooterRunVelocity(3000);
            } else {
                // Calcaulate new linear velocity
                // Get the angle to point at the goal
                var goalAngle =
                        ShooterCommands.getSpeakerPos().toPose2d()
                                .getTranslation()
                                .minus(drive.getPose().getTranslation())
                                .getAngle();
                Pose3d targetPose = ShooterCommands.getSpeakerPos();
                targetPose = targetPose.plus(new Transform3d(0.0, goalAngle.getSin() * 0.5, 0.0, new Rotation3d()));
                Pose3d shooterLocation = new Pose3d(drive.getPose()).plus(shooterOffset);
                Pose3d targetRelativeToShooter = targetPose.relativeTo(shooterLocation);
                double distance = getDistance(targetRelativeToShooter);
                double atan = atan(targetRelativeToShooter.getZ() / distance);
                Logger.recordOutput("distanceFromGoal", distance);
                Logger.recordOutput("Aim/getZ", targetRelativeToShooter.getZ());
                Logger.recordOutput("Aim/atan", atan);
                shooter.setTargetShooterAngle(Rotation2d.fromRadians(atan + distanceToAngle.get(distance)));
                shooter.overrideHoodAtSetpoint(true);
                shooter.shooterRunVelocity(distanceToRPM.get(distance));
            }
        }, shooter)
                .raceWith(SpecializedCommands.timeoutDuringAutoSim(2))
                .withName("Auto Aim");
    }

    public static Command JustShoot(Shooter shooter) {
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(.8));
            shooter.shooterRunVelocity(3500);
        }, shooter)
                .raceWith(SpecializedCommands.timeoutDuringAutoSim(2))
                .withName("Just Shoot");
    }

    public static Command shooterIdle(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(0.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.6));
        }, shooter)
                .withName("Shooter Idle");
    }

    public static Command simpleHoodZero(Shooter shooter) {
        Debouncer zeroStateDetection = new Debouncer(.2, Debouncer.DebounceType.kRising);
        return race(
                run(() -> {
                    shooter.zeroMode = true;
                    shooter.setHoodPIDEnabled(false);
                    //fixme: potentially make this higher (must do with testing, could damage robot)
                    shooter.hoodRunVolts(2);
                }, shooter),
                sequence(
                        waitSeconds(0.25).raceWith(run(() -> zeroStateDetection.calculate(
                                shooter.isStalled()
                                        || (abs(
                                        shooter.getHoodCharacterizationVelocity()
                                                .in(RadiansPerSecond)) > 1)))),
                        waitUntil(() -> !(zeroStateDetection.calculate(
                                shooter.isStalled()
                                        || (abs(
                                        shooter.getHoodCharacterizationVelocity()
                                                .in(RadiansPerSecond)) > 1)))),
                        runOnce(shooter::resetWhileZeroing),
                        runOnce(() -> shooter.setHasZeroed(true))
                )
        )
                .withTimeout(4.0)
                .finallyDo(() -> {
                    shooter.zeroMode = false;
                    shooter.setHoodPIDEnabled(true);
                })
                .withName("Simple Hood Zero");
    }

    public static Command passNote(Shooter shooter, Drive drive) {
        return run(() -> {
            shooter.shooterRunVelocity(1000);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(.25));
        }, shooter)
                .beforeStarting(() -> drive.setOverrideDriveAutoAim(true))
                .finallyDo(interrupted -> drive.setOverrideDriveAutoAim(false))
                .withName("passNote");
    }

    public static Command shooterSetZero(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(0.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0));
        }, shooter)
                .withName("shooterSetZero");
    }

    public static Command ampShoot(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(1500.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0.5166099131107331));
        }, shooter)
                .withName("Amp Shoot");
    }

    public static Command pushIntoAmp(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(100.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(-.76));
        }, shooter)
                .withName("Push note into amp");
    }

    public static Command addToOffset() {
        shooterAngleAdjustment += 0.017;
        return runOnce(() -> shooterAngleAdjustment += 0.017)
                .withName("Add to Offset");
    }

    public static Command removeFromOffset() {
        shooterAngleAdjustment -= 0.017;
        return runOnce(() -> shooterAngleAdjustment += 0.017)
                .withName("Subtract from Offset");
    }

    public static Command humanPlayerIntake(Shooter shooter){
        return run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.15));
            shooter.shooterRunVelocity(-1500);
        });
    }

    public static Command newAmpShoot(Shooter shooter){
        return run(() -> {
                shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.114));
                shooter.shooterRunVelocity(1000);
//                shooter.setTargetShooterAngle(Rotation2d.fromRadians(-0.5));
        });
    }

    public static Command ampSpin(Shooter shooter){
        return run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.114));
            shooter.shooterRunVelocity(300);
        });
    }

    public static Command ampGo(Shooter shooter, int rpm){
        return run(() -> {
            shooter.shooterRunVelocity(rpm);
        });
    }

    public static Command ampAng(Shooter shooter){
        return run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.5));
        });
    }

    public static Command ampAngle(Shooter shooter){
        return run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(-0.3));
        });
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

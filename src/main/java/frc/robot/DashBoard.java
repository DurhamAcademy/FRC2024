package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.BooleanSupplier;

public class DashBoard {
    public DashBoard(
            LoggedDashboardChooser autoChooser,
            Drive drive,
            Shooter shooter,
            Feeder feeder,
            Intake intake,
            HoodIO.HoodIOInputs hood) {
        main.addDouble("Drive Angle", () -> drive.getAngle().getDegrees())
                .withSize(3, 3)
                .withPosition(4, 4);

        main.add("Auto Chooser", autoChooser.getSendableChooser())
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 1)
                .withPosition(2, 0);

        main.addDouble("Shooter Position", () -> hood.hoodPositionRad);
        //    main.addCamera()
        main.addBoolean("Shooter Position Status", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return hood.hoodPositionRad == hood.hoodVelocityRadPerSec;
            }
        });
        main.add("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> drive.getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> drive.getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Front Right Angle", () -> drive.getModuleStates()[1].angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> drive.getModuleStates()[1].speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Left Angle", () -> drive.getModuleStates()[2].angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> drive.getModuleStates()[2].speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Right Angle", () -> drive.getModuleStates()[3].angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> drive.getModuleStates()[3].speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> drive.getRotation().getRadians(), null);
        })
                .withPosition(3, 3)
                .withSize(3, 3);
    }

    ShuffleboardTab main = Shuffleboard.getTab("Main");
}

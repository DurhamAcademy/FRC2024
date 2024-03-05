package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.VisionIOReal;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

//import static com.sun.tools.classfile.AccessFlags.Kind.Field;

public class Dashboard {
    public Dashboard(
            LoggedDashboardChooser autoChooser,
            Drive drive,
            Shooter shooter,
            Feeder feeder,
            Intake intake,
            HoodIO.HoodIOInputs hood,
            VisionIOReal vision
    ) {
        main.addDouble("Flywheel RPM", shooter::getShooterVelocityRPM)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withSize(2, 2)
                .withPosition(0, 0);

        main.add("Auto Chooser", autoChooser.getSendableChooser())
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 1)
                .withPosition(2, 0);
//        main.addCamera("Camera feed", ) FIGURE OUT NAME AND STUFF
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty("Front Left Angle", () -> drive.shuffleboardMethod()[0].getAngle().getRadians(), null);
                builder.addDoubleProperty("Front Right Angle", () -> drive.shuffleboardMethod()[1].getAngle().getRadians(), null);
                builder.addDoubleProperty("Back Left Angle", () -> drive.shuffleboardMethod()[2].getAngle().getRadians(), null);
                builder.addDoubleProperty("Back Right Angle", () -> drive.shuffleboardMethod()[3].getAngle().getRadians(), null);
                builder.addDoubleProperty("Robot Angle", () -> drive.getRotation().getRadians(), null);
            }
        });

        main.add("Hood Angle Manager", hood.hoodPositionRad)
                .withWidget(BuiltInWidgets.kEncoder);

        main.add("Field", new Field2d());
    }
    ShuffleboardTab main = Shuffleboard.getTab("Main");
}

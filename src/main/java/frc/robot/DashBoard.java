package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DashBoard {
  public DashBoard(
      LoggedDashboardChooser autoChooser,
      Drive drive,
      Shooter shooter,
      Feeder feeder,
      Intake intake,
      HoodIO.HoodIOInputs hood) {
    main.addDouble("Flywheel RPM", () -> shooter.getVelocityRPM())
        .withWidget(BuiltInWidgets.kGyro)
        .withSize(2, 2)
        .withPosition(0, 0);

    main.add("Auto Chooser", autoChooser.getSendableChooser())
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2, 1)
        .withPosition(2, 0);

    main.addDouble("Shooter Position", () -> hood.hoodPositionRad);
    //    main.addCamera()
  }

  ShuffleboardTab main = Shuffleboard.getTab("Main");
}

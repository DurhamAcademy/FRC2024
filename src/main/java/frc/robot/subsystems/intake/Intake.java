package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  ProfiledPIDController armFB;
  ArmFeedforward armFF;
  IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: add arm FF and FB
      case REPLAY:
        armFF = new ArmFeedforward(0.1, 0.075, 0.05);
        armFB =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new Constraints(RotationsPerSecond.of(3), RotationsPerSecond.per(Second).of(9)));
        break;
      case SIM:
        armFF = new ArmFeedforward(0.0, 0.075, 0.03);
        armFB =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new Constraints(RotationsPerSecond.of(3), RotationsPerSecond.per(Second).of(9)));
        break;
      default:
        armFF = new ArmFeedforward(0.0, 0.0, 0.0);
        armFB = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));
        break;
    }
  }

  // double position = 0.0;
  Rotation2d armTarget = Rotation2d.fromDegrees(0);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    io.setArmVoltage(
        armFB.calculate(inputs.armPositionRad, armTarget.getRadians())
            + armFF.calculate(inputs.armPositionRad, inputs.armVelocityRadPerSec));
  }

  public void setIntakePosition(Rotation2d position) {
    armTarget = position;
  }

  public void setRollerPercentage(double percentage) {
    io.setRollerPercent(percentage);
  }
}

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final SimpleMotorFeedforward feederFFModel;
  private final ProfiledPIDController feederFBController;
  private double feederOffset = 0.0;

  private DigitalInput feederBeambreak = new DigitalInput(0);

  public boolean getSensorFeed() {
    return feederBeambreak.get();
  }

  public Feeder(FeederIO io) {
      this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: characterize real robot
      case REPLAY:
        feederFFModel = new SimpleMotorFeedforward(0.1, 0.05);
        feederFBController =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
        break;
      case SIM:
        feederFFModel = new SimpleMotorFeedforward(0.0, 0.03);
        feederFBController =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
        break;
      default:
        feederFFModel = new SimpleMotorFeedforward(0.0, 0.0);
        feederFBController =
            new ProfiledPIDController(0., 0., .0, new TrapezoidProfile.Constraints(0., 0.));
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setFeederVoltage(
        feederFBController.calculate(inputs.feederPositionRad)
            + feederFFModel.calculate(feederFBController.getSetpoint().velocity));
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runFeederVolts(double volts) {
    io.setFeederVoltage(volts);
  }

  /** Run closed loop to the specified position. */
  public void runFeederPosition(double position) {
    feederFBController.setGoal(position + feederOffset);
    // Log flywheel setpoint
    Logger.recordOutput("Feeder/SetpointRot", position);
  }

  public void resetFeederPosition() {
    var oldGoal =
        new TrapezoidProfile.State(
            feederFBController.getGoal().position - feederOffset, feederFBController.getGoal().velocity);
    feederOffset = inputs.feederPositionRad;
    feederFBController.setGoal(new TrapezoidProfile.State(oldGoal.position + feederOffset, oldGoal.velocity));
  }

  /** Stops the flywheel. */
  public void stopFeeder() {
    io.stopFeeder();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getFeederVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.feederVelocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getFeederCharacterizationVelocity() {
    return inputs.feederVelocityRadPerSec;
  }
}

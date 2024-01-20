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
  private final SimpleMotorFeedforward ffModel;
  private final ProfiledPIDController pidController;
  double offset = 0.0;

  private DigitalInput conveyorSensor;

  private static final int conveyorSensorNum = 0;

  public boolean getSensorFeed() {
    return conveyorSensor.get();
  }

  public Feeder(FeederIO io) {
    conveyorSensor = new DigitalInput(conveyorSensorNum);
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: characterize real robot
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        pidController =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        pidController =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        pidController =
            new ProfiledPIDController(0., 0., .0, new TrapezoidProfile.Constraints(0., 0.));
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setVoltage(
        pidController.calculate(inputs.positionRad)
            + ffModel.calculate(pidController.getSetpoint().velocity));
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop to the specified position. */
  public void runPosition(double position) {
    pidController.setGoal(position + offset);
    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", position);
  }

  public void resetPosition() {
    var oldGoal =
        new TrapezoidProfile.State(
            pidController.getGoal().position - offset, pidController.getGoal().velocity);
    offset = inputs.positionRad;
    pidController.setGoal(new TrapezoidProfile.State(oldGoal.position + offset, oldGoal.velocity));
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}

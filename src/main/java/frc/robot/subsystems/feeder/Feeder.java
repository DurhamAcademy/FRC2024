package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth;
import static edu.wpi.first.units.Units.*;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final ProfiledPIDController pidController;
  double offset = 0.0;

  private State state = State.none;

  public enum State {
    feedingShooter,
    intaking,
    zeroingNote,
    humanPlayerIntake,
    none
  }

  public State getState() {
    return state;
  }

  public void setState(State state) {
    this.state = state;
  }

  Debouncer debouncer = new Debouncer(.05, kBoth);
  Debouncer debouncer2 = new Debouncer(.05, kBoth);

  @AutoLogOutput
  public boolean getBeamBroken() {
    return !debouncer.calculate(inputs.beamUnobstructed);
  }

  @AutoLogOutput
  public boolean getIntakeBeamBroken(){
    return !debouncer2.calculate(inputs.intakebeamUnobstructed);
  }

  public Feeder(FeederIO io) {
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
    if(this.getCurrentCommand() != null) {
      Logger.recordOutput("Commands/Feeder", this.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("Commands/Feeder", "");
    }
    io.updateInputs(inputs);
//    io.setVoltage(
//        pidController.calculate(inputs.positionRad)
//            + ffModel.calculate(pidController.getSetpoint().velocity));
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Run open loop at the specified voltage.
   */
  public void runVolts(Measure<Voltage> volts) {
    runVolts(volts.in(Volts));
  }

  /**
   * Run open loop at the specified voltage.
   */
  public Measure<Voltage> getCharacterizationVoltage() {
    return Volts.of(inputs.appliedVolts);
  }

  /**
   * Run open loop at the specified voltage.
   */
  public Measure<Current> getCharacterizationCurrent() {
    double sum = 0.0;
    for (int i = inputs.currentAmps.length - 1; i >= 0; i--) {
      sum += inputs.currentAmps[i];
    }
    if (inputs.currentAmps.length != 0) {
      return Amps.of(sum / inputs.currentAmps.length);
    } else return Amps.zero();
  }

  public Measure<Angle> getCharacterizationPosition() {
    return Radians.of(inputs.positionRad);
  }

  public Measure<Velocity<Angle>> getCharacterizationVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadPerSec);
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
}

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  /** Updates the set of loggable inputs. */
  default void updateInputs(FeederIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  default void setVoltage(double volts) {
  }

  /** Stop in open loop. */
  default void stop() {
  }

  @AutoLog
  class FeederIOInputs {
    public boolean beamUnobstructed = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] temperature = new double[] {};
    public boolean intakebeamUnobstructed;
  }
}

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setFeederVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stopFeeder() {}

  @AutoLog
  public static class FeederIOInputs {
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double[] feederCurrentAmps = new double[] {};
  }
}

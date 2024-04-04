
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  @AutoLog
  public static class HoodIOInputs {
    public double motorPositionRad = 0.0;
    public double hoodPositionRad = 0.0;
    public double hoodVelocityRadPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double[] hoodCurrentAmps = new double[] {};
    public double[] hoodTemperature = new double[] {};
    public boolean isStalled = false;
    public boolean islimitSwitchPressed;
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void wristStop() {}

  public default void setBrakeMode(boolean enableBrakeMode) {}
}

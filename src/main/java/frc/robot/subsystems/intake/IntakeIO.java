package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTemperature = new double[] {};

    public double rollerVelocityRadPerSec;
    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
    public double rollerPositionRad = 0.0;
    public double[] rollerTemperature = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setArmVoltage(double volts) {}

  /** Set intake wheel voltage. */
  public default void setRollerPercent(double percent) {}

  public default void setRollerVoltage(double volts) {
  }
}

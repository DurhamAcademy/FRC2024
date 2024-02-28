package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  /** Updates the set of loggable inputs. */
  default void updateInputs(ClimbIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  default void setLeftVoltage(double volts) {}

  default void setRightVoltage(double volts) {}

  /** Stop in open loop. */
  public default void leftStop() {}

  /** Stop in open loop. */
  public default void rightStop() {}

  @AutoLog
  public static class ClimbIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};
    public double[] leftTemperature = new double[] {};

    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
    public double[] rightTemperature = new double[] {};
  }
}

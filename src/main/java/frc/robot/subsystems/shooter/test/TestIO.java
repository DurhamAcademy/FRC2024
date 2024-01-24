package frc.robot.subsystems.shooter.test;

public interface TestIO {
  @AutoLog
  public static class TestIOInputs {
    public double testPositionRad = 0.0;
    public double testVelocityRadPerSec = 0.0;
    public double testAppliedVolts = 0.0;
    public double[] testCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TestIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setFlywheelVoltage(double volts) {}

  /** Stop in open loop. */
  public default void testStop() {}
}

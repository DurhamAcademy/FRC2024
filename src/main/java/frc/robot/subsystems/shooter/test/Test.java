package frc.robot.subsystems.shooter.test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase {
  private final TestIO io;
  private final TestIOInputsAutoLogged inputs = new TestIOInputsAutoLogged();

  public Test(TestIO io) {
    this.io = io;
  }

  public void periodic() {
    this.io.updateInputs(inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setFlywheelVoltage(volts);
  }
}

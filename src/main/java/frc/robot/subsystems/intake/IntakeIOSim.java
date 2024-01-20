package frc.robot.subsystems.intake;

import static java.lang.Math.PI;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim wheelSim = new FlywheelSim(DCMotor.getNEO(1), 3, .01);
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          0.13,
          Units.inchesToMeters(11.875),
          PI * .6,
          PI * -.25,
          true,
          -.2);
  private double armVoltage = 0.0;
  private double wheelVoltage = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armCurrentAmps = new double[] {armSim.getCurrentDrawAmps()};
    inputs.armAppliedVolts = armVoltage;
    inputs.armPositionRad = armSim.getAngleRads();
    inputs.armVelocityRadPerSec = armSim.getVelocityRadPerSec();

    inputs.wheelCurrentAmps = new double[] {wheelSim.getCurrentDrawAmps()};
    inputs.wheelAppliedVolts = wheelVoltage;
    inputs.wheelVelocityRadPerSec = wheelSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setArmVoltage(double volts) {
    armVoltage = volts;
    armSim.setInputVoltage(armVoltage);
  }

  @Override
  public void setRollerPercent(double percent) {
    wheelVoltage = percent * 12.0;
    wheelSim.setInputVoltage(wheelVoltage);
  }
}

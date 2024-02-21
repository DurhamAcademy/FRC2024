package frc.robot.subsystems.intake;

import static java.lang.Math.PI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim rollerSim = new FlywheelSim(DCMotor.getNEO(1), 3, .01);
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          .1819,
          Units.inchesToMeters(7.063364),
          PI * -.2,
          PI * .8,
          true,
          0); // mass is 8.495 lbs
  private double armVoltage = 0.0;
  private double rollerVoltage = 0.0;
  private Double timestamp = null;

  public IntakeIOSim() {
    setArmVoltage(0.0);
    setRollerPercent(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var ct = Timer.getFPGATimestamp();
    var dt = (timestamp == null) ? .02 : ct - timestamp;
    inputs.armCurrentAmps = new double[] {armSim.getCurrentDrawAmps()};
    inputs.armAppliedVolts = -armVoltage;
    inputs.armPositionRad = -armSim.getAngleRads();
    inputs.armVelocityRadPerSec = -armSim.getVelocityRadPerSec();
    armSim.update(0.02);

    inputs.rollerCurrentAmps = new double[] {rollerSim.getCurrentDrawAmps()};
    inputs.rollerAppliedVolts = rollerVoltage;
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    rollerSim.update(dt);
    timestamp = ct;
  }

  @Override
  public void setArmVoltage(double volts) {
    armVoltage = MathUtil.clamp(-volts, -12.0, 12.0);
    armSim.setInputVoltage(armVoltage);
  }

  @Override
  public void setRollerPercent(double percent) {
    rollerVoltage = percent * 12.0;
    rollerSim.setInputVoltage(rollerVoltage);
  }
}

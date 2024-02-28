package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class HoodIOSparkMax implements HoodIO {
  private static final double GEAR_RATIO = 1.5;

    private final CANSparkMax leader = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);

  private final SparkAbsoluteEncoder encoder =
      leader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  public HoodIOSparkMax() {
    leader.restoreFactoryDefaults();
    leader.setCANTimeout(250);
    leader.setInverted(false);
    leader.enableVoltageCompensation(12.0);
      leader.setSmartCurrentLimit(50);


      encoder.setInverted(true);

    leader.burnFlash();
  }

  public void updateInputs(HoodIOInputs inputs) {
      inputs.hoodPositionRad = MathUtil.angleModulus(encoder.getPosition() * Math.PI * 2) / GEAR_RATIO;
      inputs.hoodAppliedVolts = leader.getBusVoltage() * leader.getAppliedOutput();
      inputs.hoodCurrentAmps = new double[]{leader.getOutputCurrent()};
      inputs.hoodVelocityRadPerSec = (encoder.getVelocity() * Math.PI * 2) / GEAR_RATIO;
      inputs.hoodTemperature = new double[]{leader.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
      Logger.recordOutput("HoodVoltage", volts);
      leader.setVoltage(MathUtil.clamp(volts, -5.0, 5.0));
  }
}

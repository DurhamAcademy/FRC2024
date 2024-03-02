package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class HoodIOSparkMax implements HoodIO {
    private static final double GEAR_RATIO = 1.5;
    private static final double MOTOR_TO_ROBOT = (1 / 16.2) * Math.PI * 2;

    private final CANSparkMax leader = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);

    private final SparkAbsoluteEncoder encoder =
            leader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private final RelativeEncoder motorEncoder = leader.getEncoder();
    boolean hasReset = false;

    public HoodIOSparkMax() {
        leader.restoreFactoryDefaults();
        leader.setCANTimeout(250);
        leader.setInverted(false);
        leader.enableVoltageCompensation(12.0);
        leader.setSmartCurrentLimit(50);
        motorEncoder.setPositionConversionFactor(MOTOR_TO_ROBOT);
        motorEncoder.setVelocityConversionFactor(MOTOR_TO_ROBOT);
        motorEncoder.setPosition(encoder.getPosition());
        encoder.setInverted(true);

        leader.burnFlash();

        motorEncoder.setPositionConversionFactor(MOTOR_TO_ROBOT);
        motorEncoder.setVelocityConversionFactor(MOTOR_TO_ROBOT);
        motorEncoder.setPosition(MathUtil.angleModulus(encoder.getPosition() * Math.PI * 2) / GEAR_RATIO);
        System.out.println();
        System.out.println();
    }

  public void updateInputs(HoodIOInputs inputs) {
      if (!hasReset) {

      }
      inputs.hoodPositionRad = MathUtil.angleModulus(encoder.getPosition() * Math.PI * 2) / GEAR_RATIO;
      inputs.motorPositionRad = motorEncoder.getPosition();
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

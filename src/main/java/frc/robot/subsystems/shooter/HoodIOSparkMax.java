package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class HoodIOSparkMax implements HoodIO {
    private static final double GEAR_RATIO = 1 / 1.5;
    private static final double MOTOR_TO_ENCODER_RATIO = 1 / 10.8;

    private final CANSparkMax leader = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);

    private final SparkAbsoluteEncoder absoluteEncoder =
            leader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private final RelativeEncoder encoder = leader.getEncoder();

    private boolean ePOK = false;
    private boolean eVOK;

    public HoodIOSparkMax() {
        leader.restoreFactoryDefaults();
        leader.setCANTimeout(250);
        leader.setInverted(false);
        leader.enableVoltageCompensation(12.0);
        leader.setSmartCurrentLimit(50);

        absoluteEncoder.setInverted(true);

        encoder.setPositionConversionFactor(MOTOR_TO_ENCODER_RATIO);
        encoder.setVelocityConversionFactor(MOTOR_TO_ENCODER_RATIO);
        encoder.setPosition(absoluteEncoder.getPosition());
        encoder.setMeasurementPeriod(20);

        leader.burnFlash();
    }

  public void updateInputs(HoodIOInputs inputs) {
      inputs.hoodAbsolutePositionRad = MathUtil.angleModulus(absoluteEncoder.getPosition() * Math.PI * 2) * GEAR_RATIO;
      inputs.hoodPositionRad = encoder.getPosition();//fixme: eventually switch above line to use encoder and have encoder setup on startup
      inputs.hoodAppliedVolts = leader.getBusVoltage() * leader.getAppliedOutput();
      inputs.hoodCurrentAmps = new double[]{leader.getOutputCurrent()};
      inputs.hoodAbsoluteVelocityRadPerSec = (absoluteEncoder.getVelocity() * Math.PI * 2) * GEAR_RATIO;
      inputs.hoodVelocityRadPerSec = encoder.getVelocity();
      inputs.hoodTemperature = new double[]{leader.getMotorTemperature()};
  }

    @Override
    public void setVoltage(double volts) {
        Logger.recordOutput("HoodVoltage", volts);
        leader.setVoltage(MathUtil.clamp(volts, -5.0, 5.0));
    }
}

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodIOSparkMax implements HoodIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax leader = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);

  private final SparkAbsoluteEncoder encoder =
      leader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    public HoodIOSparkMax() {
    leader.restoreFactoryDefaults();
    leader.setCANTimeout(250);
    leader.setInverted(false);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);
    leader.burnFlash();
  }

  private Rotation2d getWristAngle() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(encoder.getPosition() * 6.28) / 1.5);
  }

    public void updateInputs(HoodIOInputs inputs) {
    inputs.wristPositionRad = encoder.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void wristStop() {
    leader.stopMotor();
  }
}

package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class FeederIOSparkMax implements FeederIO {
  private static final double GEAR_RATIO = 1.0;
  CANSparkMax feeder = new CANSparkMax(0, kBrushless);
  private static final int conveyorSensorNum = 9;
  private DigitalInput conveyorSensor;

  public FeederIOSparkMax() {
    feeder.restoreFactoryDefaults();

    feeder.setCANTimeout(250);

    feeder.setInverted(false);

    feeder.enableVoltageCompensation(12.0);
    feeder.setSmartCurrentLimit(30);

    feeder.burnFlash();

    DigitalInput
            conveyorSensor = new DigitalInput(conveyorSensorNum);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(feeder.getEncoder().getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(feeder.getEncoder().getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = feeder.getAppliedOutput() * feeder.getBusVoltage();
    inputs.currentAmps = new double[] {feeder.getOutputCurrent()};
    inputs.temperature = new double[] {feeder.getMotorTemperature()};

    inputs.beamUnobstructed = conveyorSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    feeder.setVoltage(volts);
  }

  @Override
  public void stop() {
    feeder.setVoltage(0.0);
  }
}

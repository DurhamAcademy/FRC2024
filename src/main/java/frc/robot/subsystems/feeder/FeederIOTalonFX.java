package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

public class FeederIOTalonFX implements FeederIO {

    private static final int conveyorSensorNum = 9;
    private DigitalInput conveyorSensor;
    private static final int intakeSensorNum = 7;
    private DigitalInput intakeSensor;
    private final TalonFX feedMotor = new TalonFX(43);
    private final StatusSignal<Double> feedMotorPosition = feedMotor.getPosition();
    private final StatusSignal<Double> feedMotorVelocity = feedMotor.getVelocity();
    private final StatusSignal<Double> feedMotorAppliedVolts = feedMotor.getMotorVoltage();
    private final StatusSignal<Double> feedMotorCurrent = feedMotor.getStatorCurrent();
    private final StatusSignal<Double> feedMotorTemperature = feedMotor.getDeviceTemp();

    public FeederIOTalonFX() {
        conveyorSensor = new DigitalInput(conveyorSensorNum);
        intakeSensor = new DigitalInput(intakeSensorNum);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feedMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                feedMotorPosition,
                feedMotorVelocity,
                feedMotorAppliedVolts,
                feedMotorCurrent,
                feedMotorTemperature);
        feedMotor.optimizeBusUtilization();
    }

    public void updateInputs(FeederIO.FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                feedMotorPosition,
                feedMotorVelocity,
                feedMotorAppliedVolts,
                feedMotorCurrent,
                feedMotorTemperature);
        inputs.positionRad = rotationsToRadians(feedMotorPosition.getValueAsDouble());
        inputs.velocityRadPerSec =
                rotationsToRadians(feedMotorVelocity.getValueAsDouble());
        inputs.appliedVolts = feedMotorAppliedVolts.getValueAsDouble();
        inputs.currentAmps = new double[]{feedMotorCurrent.getValueAsDouble()};
        inputs.temperature = new double[]{feedMotorTemperature.getValueAsDouble()};
        inputs.beamUnobstructed = conveyorSensor.get();
        inputs.intakebeamUnobstructed = intakeSensor.get();
    }

    @Override
    public void setVoltage(double volts) {
        feedMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        feedMotor.stopMotor();
    }
}

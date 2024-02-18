package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterIO;

public class FeederIOTalonFX implements FeederIO {
    @Override
    public void updateInputs(FeederIOInputs inputs) {
        FeederIO.super.updateInputs(inputs);
    }

    private final TalonFX feedMotor = new TalonFX(43);
    private final StatusSignal<Double> feedMotorPosition = feedMotor.getPosition();
    private final StatusSignal<Double> feedMotorVelocity = feedMotor.getVelocity();
    private final StatusSignal<Double> feedMotorAppliedVolts = feedMotor.getMotorVoltage();
    private final StatusSignal<Double> feedMotorCurrent = feedMotor.getStatorCurrent();
    private final StatusSignal<Double> feedMotorTemperature = feedMotor.getDeviceTemp();

    public FeederIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 50.0;
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
                feedMotorCurrent);
        feedMotor.optimizeBusUtilization();
    }

    public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                feedMotorPosition,
                feedMotorVelocity,
                feedMotorAppliedVolts,
                feedMotorCurrent,
                feedMotorTemperature);
        inputs.flywheelPositionRad = Units.rotationsToRadians(feedMotorPosition.getValueAsDouble());
        inputs.flywheelVelocityRadPerSec =
                Units.rotationsToRadians(feedMotorVelocity.getValueAsDouble());
        inputs.flywheelAppliedVolts = feedMotorAppliedVolts.getValueAsDouble();
        inputs.flywheelCurrentAmps = new double[] {feedMotorCurrent.getValueAsDouble()};
        inputs.flywheelTemperature = new double[] {feedMotorTemperature.getValueAsDouble()};
    }

    @Override
    public void setVoltage(double volts) {
        feedMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        feedMotor.stopMotor();
    }
}

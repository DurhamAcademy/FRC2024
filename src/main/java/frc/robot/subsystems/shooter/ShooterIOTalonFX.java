// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Value;

public class ShooterIOTalonFX implements ShooterIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX leader = new TalonFX(40);
  private final TalonFX follower = new TalonFX(41);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Double> leaderDeviceTemp = leader.getDeviceTemp();
  private final StatusSignal<Double> leaderAncillaryDeviceTemp = leader.getAncillaryDeviceTemp();
  private final StatusSignal<Double> leaderProcessorTemp = leader.getProcessorTemp();
  private final StatusSignal<Double> followerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();
  private final StatusSignal<Double> followerDeviceTemp = follower.getDeviceTemp();
  private final StatusSignal<Double> followerAncillaryDeviceTemp = follower.getAncillaryDeviceTemp();
  private final StatusSignal<Double> followerProcessorTemp = follower.getProcessorTemp();

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnConfig = false;
    config.Audio.BeepOnBoot = false;
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(20,
            leaderDeviceTemp,
            leaderAncillaryDeviceTemp,
            leaderProcessorTemp,
            followerDeviceTemp,
            followerAncillaryDeviceTemp,
            followerProcessorTemp);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent, leaderDeviceTemp,
            leaderAncillaryDeviceTemp, leaderProcessorTemp, followerDeviceTemp, followerAncillaryDeviceTemp,
            followerProcessorTemp);
    inputs.flywheelPositionRad = leaderPosition.getValueAsDouble() * GEAR_RATIO;
    inputs.flywheelVelocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) * GEAR_RATIO;
    inputs.flywheelAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.flywheelVoltages = new double[]{leaderAppliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
    inputs.flywheelCurrentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
    inputs.flywheelTemperature
            = new double[]{leaderDeviceTemp.getValueAsDouble(), followerDeviceTemp.getValueAsDouble()};
    inputs.flywheelAncillaryTemperature
            = new double[]{leaderAncillaryDeviceTemp.getValueAsDouble(), followerAncillaryDeviceTemp.getValueAsDouble()};
    inputs.flywheelProcessorTemperature
            = new double[]{leaderProcessorTemp.getValueAsDouble(), followerProcessorTemp.getValueAsDouble()};
  }

  public void playTone(Measure<Velocity<Dimensionless>> tone) {
    var musicTone = new MusicTone(tone.in(Value.per(Second)));
    leader.setControl(musicTone);
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void flywheelStop() {
    leader.stopMotor();
  }
}

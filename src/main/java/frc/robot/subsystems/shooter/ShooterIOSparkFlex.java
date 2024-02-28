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

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkFlex implements ShooterIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkFlex leader = new CANSparkFlex(0, MotorType.kBrushless);
  private final CANSparkFlex follower = new CANSparkFlex(1, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();

  public ShooterIOSparkFlex() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.flywheelAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.flywheelCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.flywheelTemperature =
        new double[] {leader.getMotorTemperature(), follower.getMotorTemperature()};
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

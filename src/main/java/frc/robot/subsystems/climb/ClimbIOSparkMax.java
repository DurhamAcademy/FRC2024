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

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
@SuppressWarnings("unused")
public class ClimbIOSparkMax implements ClimbIO {
  private static final double LEFT_GEAR_RATIO = 16.0;
  private static final double RIGHT_GEAR_RATIO = 16.0;

  private final CANSparkMax left = new CANSparkMax(20, MotorType.kBrushless);
  private final CANSparkMax right = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = left.getEncoder();
  private final RelativeEncoder rightEncoder = right.getEncoder();

  public ClimbIOSparkMax() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setCANTimeout(250);
    right.setCANTimeout(250);

    left.enableVoltageCompensation(12.0); // wait for electrical to do climb wiring
    left.setSmartCurrentLimit(30); // wait for electrical to find the right breaker for the climb

    right.enableVoltageCompensation(12.0); // wait for electrical to do climb wiring
    right.setSmartCurrentLimit(30); // wait for electrical to find the right breaker for the climb

    left.setIdleMode(CANSparkBase.IdleMode.kBrake);
    right.setIdleMode(CANSparkBase.IdleMode.kBrake);

    left.burnFlash();
    right.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / LEFT_GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / LEFT_GEAR_RATIO);
    inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.leftCurrentAmps = new double[] {left.getOutputCurrent()};
    inputs.leftTemperature = new double[] {left.getMotorTemperature()};
    inputs.rightPositionRad =
        Units.rotationsToRadians(rightEncoder.getPosition() / RIGHT_GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / RIGHT_GEAR_RATIO);
    inputs.rightAppliedVolts = right.getAppliedOutput() * right.getBusVoltage();
    inputs.rightCurrentAmps = new double[] {right.getOutputCurrent()};
    inputs.rightTemperature = new double[] {right.getMotorTemperature()};
  }

  // sets the left motor voltage
  public void setLeftVoltage(double volts) {
    left.setVoltage(volts);
  }

  // sets the right motor voltage
  public void setRightVoltage(double volts) {
    right.setVoltage(volts);
  }
}

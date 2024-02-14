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

package frc.robot.subsystems.intake;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double ARM_GEAR_RATIO = 100.0;
  private static final double ROLLER_GEAR_RATIO = 3.0;

  private final CANSparkMax arm = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax roller = new CANSparkMax(1, MotorType.kBrushless);
  private final AbsoluteEncoder encoder =
      arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final SparkPIDController pid = arm.getPIDController();

  public IntakeIOSparkMax() {
    arm.restoreFactoryDefaults();
    roller.restoreFactoryDefaults();

    arm.setCANTimeout(250);
    roller.setCANTimeout(250);

    arm.setInverted(false);
    roller.follow(arm, false);

    arm.enableVoltageCompensation(12.0);
    roller.setSmartCurrentLimit(30);

    arm.burnFlash();
    roller.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armPositionRad = Units.rotationsToRadians(encoder.getPosition() / ARM_GEAR_RATIO);
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / ARM_GEAR_RATIO);
    inputs.armAppliedVolts = arm.getAppliedOutput() * arm.getBusVoltage();
    inputs.armCurrentAmps = new double[] {arm.getOutputCurrent(), roller.getOutputCurrent()};
    inputs.armTemperature = new double[] {arm.getMotorTemperature()};
    inputs.rollerTemperature = new double[] {roller.getMotorTemperature()};
  }

  public void setVoltage(double volts) {
    arm.setVoltage(volts);
  }
  //
  //    @Override
  //    public void setArmVelocity(double velocityRadPerSec, double ffVolts) {
  //        pid.setReference(
  //                Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * ARM_GEAR_RATIO,
  //                ControlType.kVelocity,
  //                0,
  //                ffVolts,
  //                ArbFFUnits.kVoltage);
  //    }
  //
  //    @Override
  //    public void stop() {
  //        arm.stopMotor();
  //    }
  //
  //    @Override
  //    public void configurePID(double kP, double kI, double kD) {
  //        pid.setP(kP, 0);
  //        pid.setI(kI, 0);
  //        pid.setD(kD, 0);
  //        pid.setFF(0, 0);
  //    }
}

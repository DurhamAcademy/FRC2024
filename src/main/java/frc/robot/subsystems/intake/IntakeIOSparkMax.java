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

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double ARM_GEAR_RATIO = 100.0;
  private static final double ROLLER_GEAR_RATIO = 3.0;

  private final CANSparkMax arm = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax roller = new CANSparkMax(31, MotorType.kBrushless);
  private final AbsoluteEncoder encoder =
          arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final AbsoluteEncoder rollerEncoder =
          arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  public IntakeIOSparkMax() {
    arm.restoreFactoryDefaults();
    roller.restoreFactoryDefaults();
    arm.setIdleMode(CANSparkBase.IdleMode.kBrake);

    arm.setCANTimeout(250);
    roller.setCANTimeout(250);

    arm.setInverted(true);
    arm.enableVoltageCompensation(12.0);
    arm.setSmartCurrentLimit(30);

    roller.setInverted(false);
    roller.enableVoltageCompensation(12.0);
    roller.setSmartCurrentLimit(30);

    encoder.setInverted(false);

    arm.burnFlash();
    roller.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armPositionRad = MathUtil.angleModulus(Units.rotationsToRadians(encoder.getPosition()));
    inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.armAppliedVolts = arm.getAppliedOutput() * arm.getBusVoltage();
    inputs.armCurrentAmps = new double[] {arm.getOutputCurrent()};
    inputs.armTemperature = new double[] {arm.getMotorTemperature()};

    inputs.rollerPositionRad = MathUtil.angleModulus(Units.rotationsToRadians(rollerEncoder.getPosition()));
    inputs.rollerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rollerEncoder.getVelocity());
    inputs.rollerAppliedVolts = roller.getAppliedOutput() * roller.getBusVoltage();
    inputs.rollerCurrentAmps = new double[]{roller.getOutputCurrent()};
    inputs.rollerTemperature = new double[] {roller.getMotorTemperature()};
  }

  @Override
  public void setArmVoltage(double volts) {
    Logger.recordOutput("ArmSetVoltage", volts);
    arm.setVoltage(MathUtil.clamp(volts, -10, 10));
  }

  @Override
  public void setRollerPercent(double percent) {
    roller.setVoltage(percent * roller.getBusVoltage());
  }

  public void setRollerVoltage(double volts) {
    roller.setVoltage(volts);
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

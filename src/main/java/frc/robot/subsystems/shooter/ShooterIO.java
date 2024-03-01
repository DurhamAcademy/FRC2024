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

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelPositionRad = 0.0;
    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double[] flywheelVoltages = new double[]{};
    public double[] flywheelCurrentAmps = new double[] {};
    public double[] flywheelTemperature = new double[] {};
    public double[] flywheelAncillaryTemperature = new double[]{};
    public double[] flywheelProcessorTemperature = new double[]{};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setFlywheelVoltage(double volts) {}

  /** Stop in open loop. */
  public default void flywheelStop() {}
}

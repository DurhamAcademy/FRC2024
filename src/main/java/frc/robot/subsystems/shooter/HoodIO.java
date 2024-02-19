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

public interface HoodIO {
  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  @AutoLog
  public static class HoodIOInputs {
    public double hoodPositionRad = 0.0;
    public double hoodVelocityRadPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double[] hoodCurrentAmps = new double[] {};
    public double[] hoodTemperature = new double[] {};
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void wristStop() {}
}

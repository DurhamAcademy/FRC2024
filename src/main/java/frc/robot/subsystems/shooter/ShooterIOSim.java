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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim sim = new FlywheelSim(DCMotor.getKrakenX60(2), 1.5, 0.004);
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          9,
          .1,
          .2,
          Radians.convertFrom(90, Degrees),
          Radians.convertFrom(-5, Degrees),
          true,
          Radians.convertFrom(45, Degrees));
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(0.02);

    inputs.flywheelVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.flywheelAppliedVolts = appliedVolts;
    inputs.flywheelCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void flywheelStop() {
    setFlywheelVoltage(0.0);
  }
}

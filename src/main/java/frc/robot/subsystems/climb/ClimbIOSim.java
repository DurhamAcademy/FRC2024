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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;

public class ClimbIOSim implements ClimbIO {
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), 1.5, 0.025, 0.1, -Math.PI / 4, Math.PI / 2, true, 0);
  //  private ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0);
  //  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    if (closedLoop) {
      //      appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0,
      // 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.leftPositionRad = sim.getAngleRads();
    inputs.leftVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.leftAppliedVolts = appliedVolts;
    inputs.leftCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.rightPositionRad = sim.getAngleRads();
    inputs.rightVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.rightAppliedVolts = appliedVolts;
    inputs.rightCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setLeftVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }
}

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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

@SuppressWarnings("unused")
public class ClimbIOSim implements ClimbIO {

  private DCMotorSim leftSim = new DCMotorSim(DCMotor.getNEO(1), 16.0, 0.025);
  private DCMotorSim rightSim = new DCMotorSim(DCMotor.getNEO(1), 16.0, 0.025);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  //  private ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0);
  //  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0);

  private boolean closedLoop = false;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    if (closedLoop) {
      //      appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0,
      // 12.0);
      leftSim.setInputVoltage(leftAppliedVolts);
      rightSim.setInputVoltage(rightAppliedVolts);
    }

    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftPositionRad = leftSim.getAngularPositionRad();
    inputs.leftPosition = new Rotation2d(leftSim.getAngularPositionRad());
    inputs.leftVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {Math.abs(leftSim.getCurrentDrawAmps())};
    inputs.rightPositionRad = rightSim.getAngularPositionRad();
    inputs.rightPosition = new Rotation2d(rightSim.getAngularPositionRad());
    inputs.rightVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {Math.abs(rightSim.getCurrentDrawAmps())};
  }

  @Override
  public void setLeftVoltage(double volts) {
    closedLoop = false;
    leftAppliedVolts = 0.0;
    leftSim.setInputVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    closedLoop = false;
    rightAppliedVolts = 0.0;
    rightSim.setInputVoltage(volts);
  }
}

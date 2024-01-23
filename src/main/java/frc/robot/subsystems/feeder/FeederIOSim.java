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

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class FeederIOSim implements FeederIO {
    private SingleJointedArmSim feederSim =
            new SingleJointedArmSim(
                    DCMotor.getNEO(1), 1.5, 0.025, 0.1, -Math.PI / 4, Math.PI / 2, true, 0);
    //  private ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0);
    //  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederSim.setInputVoltage(appliedVolts);

        feederSim.update(0.02);

        inputs.feederPositionRad = feederSim.getAngleRads();
        inputs.feederVelocityRadPerSec = feederSim.getVelocityRadPerSec();
        inputs.feederAppliedVolts = appliedVolts;
        inputs.feederCurrentAmps = new double[]{feederSim.getCurrentDrawAmps()};
    }

    @Override
    public void setFeederVoltage(double volts) {
        appliedVolts = 0.0;
        feederSim.setInputVoltage(volts);
    }

    @Override
    public void stopFeeder() {
        setFeederVoltage(0.0);
    }
}

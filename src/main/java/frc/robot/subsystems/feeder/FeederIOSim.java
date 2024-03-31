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
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.function.BooleanSupplier;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim sim =
            new FlywheelSim(
                    DCMotor.getNEO(1), 1.5, 0.025);
    //  private ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0);
    //  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0);

    private int cycles = 0;

    private BooleanSupplier intakeSupplier = () -> cycles % 512 == 0;
    private BooleanSupplier shooterSupplier = () -> cycles % 848 == 0;

    public FeederIOSim(BooleanSupplier intakeSupplier, BooleanSupplier shooterSupplier) {
        this.intakeSupplier = intakeSupplier;
        this.shooterSupplier = shooterSupplier;
    }

    public FeederIOSim() {}

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        cycles++;
        sim.setInputVoltage(appliedVolts);

        sim.update(0.02);

        inputs.positionRad = sim.getAngularVelocityRadPerSec();
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[]{sim.getCurrentDrawAmps()};
        inputs.temperature = new double[]{0.0};
        inputs.beamUnobstructed = shooterSupplier.getAsBoolean();
        inputs.intakebeamUnobstructed = intakeSupplier.getAsBoolean();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        sim.setState(sim.getAngularVelocityRadPerSec());
        setVoltage(0.0);
    }
}

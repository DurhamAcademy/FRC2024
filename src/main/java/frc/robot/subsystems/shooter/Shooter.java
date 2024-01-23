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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ProfiledPIDController pid;
    private final SimpleMotorFeedforward ffModel;
    private static final double ENCODER_ANGLE_FIX = 1.0;
    private static double targetShooterAngleRad = 0.0;

    private final SimpleMotorFeedforward feederFFModel;
    private final ProfiledPIDController feederFBController;
    private double feederOffset = 0.0;

    private DigitalInput feederBeambreak = new DigitalInput(0);

    public boolean getSensorFeed() {
        return feederBeambreak.get();
    }

    /**
     * Creates a new Shooter.
     */
    public Shooter(ShooterIO io) {
        this.io = io;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.currentMode) {
            case REAL:
                // FIXME: characterize real robot
            case REPLAY:
                feederFFModel = new SimpleMotorFeedforward(0.1, 0.05);
                feederFBController =
                        new ProfiledPIDController(
                                1.0,
                                0.0,
                                0.0,
                                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration

                ffModel = new SimpleMotorFeedforward(0.1, 0.05);
                pid =
                        new ProfiledPIDController(
                                1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.5, 99)); // FIXME: remove profile?
                break;
            case SIM:
                feederFFModel = new SimpleMotorFeedforward(0.0, 0.03);
                feederFBController =
                        new ProfiledPIDController(
                                1.0,
                                0.0,
                                0.0,
                                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
                ffModel = new SimpleMotorFeedforward(0.0, 0.03);
                pid = new ProfiledPIDController(0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(0.5, 99));
                break;
            default:
                feederFFModel = new SimpleMotorFeedforward(0.0, 0.0);
                feederFBController =
                        new ProfiledPIDController(0., 0., .0, new TrapezoidProfile.Constraints(0., 0.));
                ffModel = new SimpleMotorFeedforward(0.0, 0.0);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        io.setFlywheelVoltage(
                pid.calculate(inputs.flywheelVelocityRadPerSec)
                        + ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
        targetShooterAngleRad = pid.getSetpoint().position * ENCODER_ANGLE_FIX;
        Logger.processInputs("Shooter", inputs);
        io.setFeederVoltage(
                feederFBController.calculate(inputs.feederPositionRad)
                        + feederFFModel.calculate(feederFBController.getSetpoint().velocity));
        Logger.processInputs("Flywheel", inputs);
    }

    /**
     * Run open loop at the specified voltage.
     */
    public void runVolts(double volts) {
        io.setFlywheelVoltage(volts);
    }

    /**
     * Run closed loop at the specified velocity.
     */
    public void runVelocity(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

        pid.setGoal(velocityRadPerSec);

        // Log flywheel setpoint
        Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
    }

    /**
     * Stops the flywheel.
     */
    public void stop() {
        io.flywheelStop();
    }

    /**
     * Returns the current velocity in RPM.
     */
    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.flywheelVelocityRadPerSec);
    }

    /**
     * Returns the current velocity in radians per second.
     */
    public double getCharacterizationVelocity() {
        return inputs.flywheelVelocityRadPerSec;
    }

    /**
     * Run open loop at the specified voltage.
     */
    public void runFeederVolts(double volts) {
        io.setFeederVoltage(volts);
    }

    /**
     * Run closed loop to the specified position.
     */
    public void runFeederPosition(double position) {
        feederFBController.setGoal(position + feederOffset);
        // Log flywheel setpoint
        Logger.recordOutput("Feeder/SetpointRot", position);
    }

    public void resetFeederPosition() {
        var oldGoal =
                new TrapezoidProfile.State(
                        feederFBController.getGoal().position - feederOffset, feederFBController.getGoal().velocity);
        feederOffset = inputs.feederPositionRad;
        feederFBController.setGoal(new TrapezoidProfile.State(oldGoal.position + feederOffset, oldGoal.velocity));
    }

    /**
     * Stops the flywheel.
     */
    public void stopFeeder() {
        io.stopFeeder();
    }

    /**
     * Returns the current velocity in RPM.
     */
    @AutoLogOutput
    public double getFeederVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.feederVelocityRadPerSec);
    }

    /**
     * Returns the current velocity in radians per second.
     */
    public double getFeederCharacterizationVelocity() {
        return inputs.feederVelocityRadPerSec;
    }
}

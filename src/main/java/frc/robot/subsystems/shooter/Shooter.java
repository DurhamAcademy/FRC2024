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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    // the ratio for turning the shooter
//    private static final double TURN_SHOOTER_RATIO = 5.4;
    private static double targetHoodAngleRad = 0.0;
    private final ShooterIO shooterIO;
    private final HoodIO hoodIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
    private PIDController shooterVelocityFB;
    double setpointRadPS = 0;
    private ProfiledPIDController hoodFB;
    private SimpleMotorFeedforward shooterVelocityFF;
    private boolean characterizeMode = false;
    Mechanism2d mech1 = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech1.getRoot("shooter", 0, 0);
    MechanismLigament2d sState = root.append(new MechanismLigament2d("shooter", 0.14, -90.0));

    private Rotation2d hoodOffsetAngle = new Rotation2d();

    private boolean hasReset = false;
    private boolean hoodPIDEnabled = true;

    public void setHoodPIDEnabled(boolean hoodPIDEnabled) {
        this.hoodPIDEnabled = hoodPIDEnabled;
    }

    double previousAnglularVelocity = 0.0;

    /**
     * Creates a new Shooter.
     */
    public Shooter(ShooterIO io, HoodIO hoodIO) {
        this.shooterIO = io;
        this.hoodIO = hoodIO;
        sState.setLength(0.14); //im not sure if this should be here

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.currentMode) {
            case REAL:
                hoodFB = new ProfiledPIDController(6.0, 0.0, .25, new TrapezoidProfile.Constraints(1912, 7600.0 / 32.0));
                hoodFB.setTolerance(0.1);
                shooterVelocityFB =
                        new PIDController(0.0079065, 0.0, 0.0);
                shooterVelocityFB.setTolerance(218.69 * .25); // this is the pid max velocity error (rad/sec)
                shooterVelocityFF = new SimpleMotorFeedforward(.58287, .013052, .0038592);
                break;
            case REPLAY:
                hoodFB = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));

                shooterVelocityFB =
                        new PIDController(0.0050812, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/);
                shooterVelocityFB.setTolerance(25);
                shooterVelocityFF = new SimpleMotorFeedforward(0.10548, 0.11959, 0.066251);
                break;
            case SIM:
                shooterVelocityFB =
                        new PIDController(0.5, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/);
                hoodFB = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));
                shooterVelocityFF = new SimpleMotorFeedforward(0, 0);
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Hood", hoodInputs);

        if (!hasReset) {
            hoodFB.reset(hoodInputs.motorPositionRad);
            hasReset = true;
        }

        if (!characterizeMode) {
            shooterIO.setFlywheelVoltage(
                    shooterVelocityFB.calculate(shooterInputs.flywheelVelocityRadPerSec, setpointRadPS)
                            + this.shooterVelocityFF.calculate(shooterVelocityFB.getSetpoint()));
        }
        Logger.recordOutput("Shooter/ShooterSpeed", setpointRadPS);
        Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodAngleRad);
        Logger.recordOutput("Shooter/HoodInputs/OffsetMotorPositionRad", hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians());

        previousAnglularVelocity = hoodInputs.hoodVelocityRadPerSec;
        if (previousAnglularVelocity != 0.0)
            Logger.recordOutput(
                    "Shooter/HoodAcceleration",
                    RadiansPerSecond.of(previousAnglularVelocity - hoodInputs.hoodVelocityRadPerSec).per(Seconds.of(0.02)));
        if (hoodPIDEnabled) {
            hoodIO.setVoltage(
                    hoodFB.calculate(hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians(), targetHoodAngleRad)
            );
        }
        sState.setAngle(hoodInputs.hoodPositionRad);
        Logger.recordOutput("shooter", mech1);
    }

    public void resetToStartingAngle() {
        hoodOffsetAngle = new Rotation2d(hoodInputs.motorPositionRad - 1.98542);
    }

    @AutoLogOutput
    public boolean flywheelAtSetpoint() {
        return this.shooterVelocityFB.atSetpoint();
    }

    public boolean hoodAtSetpoint() {
        return this.hoodFB.atGoal();
    }

    @AutoLogOutput
    public boolean allAtSetpoint() {
        return flywheelAtSetpoint() && hoodAtSetpoint();
    }

    public void setCharacterizeMode(boolean on) {
        characterizeMode = on;
    }

    /**
     * Run open loop at the specified voltage.
     */
    public void shooterRunVolts(Measure<Voltage> voltage) {
        shooterIO.setFlywheelVoltage(voltage.in(Volts));
    }

    public Measure<Voltage> getCharacterizationAppliedVolts() {
        return Volts.of(this.shooterInputs.flywheelAppliedVolts);
    }

    public void hoodRunVolts(double volts) {
        hoodIO.setVoltage(volts);
    }

    /**
     * Run closed loop at the specified velocity.
     */
    public void shooterRunVelocity(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

        setpointRadPS = velocityRadPerSec;

        // Log flywheel setpoint
        Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
    }

    /**
     * Stops the flywheel.
     */
    public void stopShooter() {
        shooterIO.flywheelStop();
    }

    public void stopHood() {
        hoodIO.wristStop();
    }

    /**
     * Returns the current velocity in RPM.
     */
    @AutoLogOutput
    public double getShooterVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(shooterInputs.flywheelVelocityRadPerSec);
    }

    /**
     * Returns the current velocity in radians per second.
     */
    public Measure<Velocity<Angle>> getCharacterizationVelocity() {
        return RadiansPerSecond.of(shooterInputs.flywheelVelocityRadPerSec);
    }

    /**
     * Returns the current velocity in radians per second.
     */
    public Measure<Angle> getCharacterizationPosition() {
        return Radians.of(shooterInputs.flywheelPositionRad);
    }

    /**
     * Returns the current velocity in radians per second.
     */
    public Measure<Current> getCharacterizationCurrent() {
        var sum = 0.0;
        for (double flywheelCurrentAmp : shooterInputs.flywheelCurrentAmps) sum += flywheelCurrentAmp;

        sum = (shooterInputs.flywheelCurrentAmps.length > 0) ? sum / shooterInputs.flywheelCurrentAmps.length : 0.0;
        return Amps.of(sum);
    }

    public void setTargetShooterAngle(Rotation2d angle) {
        targetHoodAngleRad = MathUtil.clamp(angle.getRadians(), -2, 2);
    }

    public Measure<Voltage> getHoodCharacterizationVoltage() {
        return Volts.of(hoodInputs.hoodAppliedVolts);
    }

    public Measure<Angle> getHoodCharacterizationPosition() {
        return Radians.of(hoodInputs.hoodPositionRad);
    }

    public Measure<Velocity<Angle>> getHoodCharacterizationVelocity() {
        return RadiansPerSecond.of(hoodInputs.hoodVelocityRadPerSec);
    }

    public void runHoodVoltage(Measure<Voltage> voltageMeasure) {
        hoodRunVolts(voltageMeasure.in(Volts));
    }
}

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotState.isDisabled;

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

    private boolean hasZeroed = false;

    public boolean hasZeroed() {
        return hasZeroed;
    }

    public void setHasZeroed(boolean hasZeroed) {
        this.hasZeroed = hasZeroed;
    }

    public boolean getLimitSwitch() {
        return hoodInputs.islimitSwitchPressed;
    }

    private boolean hasReset = false;
    private boolean hoodPIDEnabled = true;

    public void setHoodPIDEnabled(boolean hoodPIDEnabled) {
        this.hoodPIDEnabled = hoodPIDEnabled;
    }

    double previousAnglularVelocity = 0.0;

    private boolean lastLimitSwitch = true;

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
                hoodFB = new ProfiledPIDController(6.0, 0.0, .25, new TrapezoidProfile.Constraints(1000.0 / 2.0, 7600.0 / 32.0));
                hoodFB.setTolerance(0.025);
                shooterVelocityFB =
                        new PIDController(0.0079065 * 5, 0.0015, 0.0);
                shooterVelocityFB.setIZone(2);
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

    public boolean zeroMode = false;

    @Override
    public void periodic() {
        if(this.getCurrentCommand() != null) {
            Logger.recordOutput("Commands/Shooter", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Commands/Shooter", "");
        }
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter/Flywheel", shooterInputs);

        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Shooter/Hood", hoodInputs);


        if (!hasReset) {
            resetToStartingAngle();
            hoodFB.reset(hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians());
            hasReset = true;
        }

        if (!characterizeMode || setpointRadPS != 0.0) {
            if (setpointRadPS == 0.0) shooterIO.setFlywheelVoltage(0.0);
            else
                shooterIO.setFlywheelVoltage(
                        shooterVelocityFB.calculate(shooterInputs.flywheelVelocityRadPerSec, setpointRadPS)
                                + this.shooterVelocityFF.calculate(shooterVelocityFB.getSetpoint()));
        } else if (setpointRadPS == 0.0) shooterIO.setFlywheelVoltage(0.0);
        Command currentCommand = getCurrentCommand();
        if (currentCommand != null)
            Logger.recordOutput("Shooter/Current Command", currentCommand.getName());
        else Logger.recordOutput("Shooter/Current Command", "null");
        Logger.recordOutput("Shooter/Shooter Speed", setpointRadPS);
        Logger.recordOutput("Shooter/Hood/Target Hood Angle", targetHoodAngleRad);
        Logger.recordOutput("Shooter/Hood/Inputs/Offset Motor Position Radians", hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians());
        Logger.recordOutput("Shooter/Hood/Offset Radians", hoodOffsetAngle.getRadians());
        Logger.recordOutput("Shooter/Hood/Beam Break Status", hoodInputs.islimitSwitchPressed);

        previousAnglularVelocity = hoodInputs.hoodVelocityRadPerSec;
        if (previousAnglularVelocity != 0.0)
            Logger.recordOutput(
                    "Shooter/Hood Acceleration",
                    RadiansPerSecond.of(previousAnglularVelocity - hoodInputs.hoodVelocityRadPerSec).per(Seconds.of(0.02)));
        if (hoodPIDEnabled) {
            hoodIO.setVoltage(
                    hoodFB.calculate(hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians(), targetHoodAngleRad)
            );
        }
        sState.setAngle(hoodInputs.hoodPositionRad);
        Logger.recordOutput("Shooter/Mechanism", mech1);

        // Let the hood move more easily while disabled so that we don't skip gears as much
        hoodIO.setBrakeMode(isDisabled() && (Math.abs(shooterInputs.flywheelVelocityRadPerSec) < 1000));

        if (hoodInputs.islimitSwitchPressed != lastLimitSwitch) {
            lastLimitSwitch = hoodInputs.islimitSwitchPressed;
            // only do it during these times to prevent arbitrary zero events
            if (zeroMode || isDisabled())
                resetToLimitAngle();
        }
    }

    public void resetToLimitAngle(){
        hoodOffsetAngle = new Rotation2d(hoodInputs.motorPositionRad - (1.98875));
        hoodFB.reset(hoodInputs.motorPositionRad - (1.98875+.16));
    }

    public void resetToStartingAngle() {
        hoodOffsetAngle = new Rotation2d(hoodInputs.motorPositionRad - 1.98542);
        hoodFB.reset(hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians());
    }

    public void resetWhileZeroing() {
        hoodOffsetAngle = new Rotation2d(hoodInputs.motorPositionRad - (2.225));
        hoodFB.reset(hoodInputs.motorPositionRad - hoodOffsetAngle.getRadians());
    }

    boolean hoodOverride = true;

    public void overrideHoodAtSetpoint(boolean isAtSetpoint) {
        hoodOverride = isAtSetpoint;
    }

    public boolean isStalled() {
        return hoodInputs.isStalled;
    }

    @AutoLogOutput
    public boolean flywheelAtSetpoint() {
        return this.shooterVelocityFB.atSetpoint();
    }

    @AutoLogOutput
    public boolean hoodAtSetpoint() {
        return this.hoodFB.atGoal() && hoodOverride;
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

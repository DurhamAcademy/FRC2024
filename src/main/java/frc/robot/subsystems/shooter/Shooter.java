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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
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
    private final ArmFeedforward hoodFF;
    private PIDController shooterVelocityFB;
    double setpointRadPS = 0;
    private ProfiledPIDController hoodFB;
    //    private ProfiledPIDController shooterPositionFB;
    private SimpleMotorFeedforward shooterVelocityFF;
    private boolean characterizeMode = false;

    /**
     * Creates a new Shooter.
     */
    public Shooter(ShooterIO io, HoodIO hoodIO) {
        this.shooterIO = io;
        this.hoodIO = hoodIO;
        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.currentMode) {
            case REAL:
                hoodFB = new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));
//                if (shooterIO instanceof ShooterIOTalonFX) {
                    shooterVelocityFB =
                            new PIDController(0.0010812, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/);
                    shooterVelocityFB.setTolerance(25);
//                    shooterPositionFB =
//                            new ProfiledPIDController(20.817, 0.0, 1.7743, new TrapezoidProfile.Constraints(RadiansPerSecond.of(90), RadiansPerSecond.per(Second).of(27.8736842105)));
//                    shooterPositionFB.setTolerance(0.26, 5);
                shooterVelocityFF = new SimpleMotorFeedforward(.998, .0706 * .5, .16);
//                }
                // FIXME: characterize real robot

                hoodFF = new ArmFeedforward(0.0, 0.0, 0);
                break;
            case REPLAY:
                hoodFF = new ArmFeedforward(0.10548, 0.05, 0);
                hoodFB = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));

                shooterVelocityFB =
                        new PIDController(0.0050812, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/);
                shooterVelocityFB.setTolerance(25);
//                shooterPositionFB =
//                        new ProfiledPIDController(20.817, 0.0, 1.7743, new TrapezoidProfile.Constraints(RadiansPerSecond.of(90), RadiansPerSecond.per(Second).of(27.8736842105)));
//                shooterPositionFB.setTolerance(0.26, 5);
                shooterVelocityFF = new SimpleMotorFeedforward(0.10548, 0.11959, 0.066251);
                break;
            case SIM:
                hoodFF = new ArmFeedforward(0.0, 0.0, 0.03);
                shooterVelocityFB =
                        new PIDController(0.5, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/);
                hoodFB = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));
                shooterVelocityFF = new SimpleMotorFeedforward(0, 0);
                break;
            default:
                hoodFF = new ArmFeedforward(0.0, 0.0, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        hoodIO.updateInputs(hoodInputs);
        if (!characterizeMode) {
            shooterIO.setFlywheelVoltage(
                    shooterVelocityFB.calculate(shooterInputs.flywheelVelocityRadPerSec, setpointRadPS)
                            + this.shooterVelocityFF.calculate(shooterVelocityFB.getSetpoint()));
        }
        Logger.recordOutput("shooterSpeed", setpointRadPS);
        Logger.recordOutput("targetHoodAngle", targetHoodAngleRad);
        Logger.recordOutput("hoodInputs.hoodPositionRad", hoodInputs.hoodPositionRad);
        Logger.recordOutput("pidStuff", "" + hoodFB.getD() + " " + hoodFB.getI() + " " + hoodFB.getP());
        hoodIO.setVoltage(
                hoodFB.calculate(hoodInputs.hoodPositionRad, targetHoodAngleRad)
                /*+ hoodFF.calcula te(hoodFB.getSetpoint().position, hoodFB.getSetpoint().velocity)*/);
//        targetHoodAngleRad =
//                hoodInputs.hoodPositionRad * HOOD_ENCODER_ANGLE_TO_REAL_ANGLE_RATIO
//                        + HOOD_ENCODER_ANGLE_TO_REAL_ANGLE_OFFSET;
        Logger.processInputs("Shooter", shooterInputs);
        Logger.processInputs("Hood", hoodInputs);
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

    /**
     * Run open loop at the specified voltage.
     */
    public void shooterRunVolts(double volts) {
        shooterIO.setFlywheelVoltage(volts);
    }

    public Measure<Voltage> getCharacterizationAppliedVolts() {
        return Volts.of(this.shooterInputs.flywheelAppliedVolts);
    }

    public void hoodRunVolts(double volts) {
        hoodIO.setVoltage(volts);
    }

    /**
     *
   * Run open loop at the specified voltage.
   */
  public void runVoltage(Measure<Voltage> voltage) {
      shooterRunVolts(voltage.in(Volts));
  }

  /** Run closed loop at the specified velocity.
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

  /** Returns the current velocity in radians per second. */
  public Measure<Velocity<Angle>> getCharacterizationVelocity() {
      return RadiansPerSecond.of(shooterInputs.flywheelVelocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public Measure<Angle> getCharacterizationPosition() {
      return Radians.of(shooterInputs.flywheelPositionRad);
  }

  /** Returns the current velocity in radians per second. */
  public Measure<Voltage> getCharacterizationVoltage() {
      return Volts.of(shooterInputs.flywheelAppliedVolts);
    }

    /** Returns the current velocity in radians per second. */
  public Measure<Current> getCharacterizationCurrent() {
    var sum = 0.0;
      for (double flywheelCurrentAmp : shooterInputs.flywheelCurrentAmps) sum += flywheelCurrentAmp;

      sum = (shooterInputs.flywheelCurrentAmps.length > 0) ? sum / shooterInputs.flywheelCurrentAmps.length : 0.0;
    return Amps.of(sum);
  }

  public void setTargetShooterAngle(Rotation2d anglediff) {
        targetHoodAngleRad = MathUtil.clamp(anglediff.getRadians(), -2, 2);
    }
}

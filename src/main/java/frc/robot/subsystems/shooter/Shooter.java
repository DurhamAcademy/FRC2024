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
  private static final double HOOD_ENCODER_ANGLE_TO_REAL_ANGLE_RATIO = 1.5;
  private static final double HOOD_ENCODER_ANGLE_TO_REAL_ANGLE_OFFSET = 0.0;
  // the ratio for turning the shooter
  private static final double TURN_SHOOTER_RATIO = 5.4;
  private static double targetHoodAngleRad = 0.0;
  private final ShooterIO shooterIO;
  private final HoodIO hoodIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final ArmFeedforward hoodFF;
  private PIDController shooterVelocityFB;
  private SimpleMotorFeedforward shooterVelocityFF;
  private ProfiledPIDController hoodFB;
  private boolean characterizeMode;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io, HoodIO hoodIO) {
    this.shooterIO = io;
    this.hoodIO = hoodIO;
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: characterize real robot
      case REPLAY:
        hoodFF = new ArmFeedforward(0.1, 0.05, 0);
        shooterVelocityFB =
                new PIDController(1.0, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/);
        hoodFB = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));
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
    if (characterizeMode) {
      shooterIO.setFlywheelVoltage(
              shooterVelocityFB.calculate(shooterInputs.flywheelVelocityRadPerSec)
                      + this.shooterVelocityFF.calculate(shooterInputs.flywheelVelocityRadPerSec));
    }
    hoodIO.setVoltage(hoodFB.calculate(hoodInputs.armPositionRad, hoodInputs.armVelocityRadPerSec));
    targetHoodAngleRad =
            hoodInputs.armPositionRad * HOOD_ENCODER_ANGLE_TO_REAL_ANGLE_RATIO
                    + HOOD_ENCODER_ANGLE_TO_REAL_ANGLE_OFFSET;
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Hood", hoodInputs);
  }

  public void setCharacterizeMode(boolean on) {
    characterizeMode = on;
  }

  /** Run open loop at the specified voltage. */
  public void shooterRunVolts(Measure<Voltage> voltage) {
    shooterIO.setFlywheelVoltage(voltage.in(Volts));
  }

  /**
   * Run open loop at the specified voltage.
   */
  public void shooterRunVolts(double volts) {
    shooterIO.setFlywheelVoltage(volts);
  }

  public Measure<Velocity<Angle>> getCharacterizationVelocity() {
    return RadiansPerSecond.of(this.shooterInputs.flywheelVelocityRadPerSec);
  }

  public Measure<Current> getCharacterizationCurrent() {
    return Amps.of(this.shooterInputs.flywheelCurrentAmps[0]);
  }

  public Measure<Voltage> getCharacterizationAppliedVolts() {
    return Volts.of(this.shooterInputs.flywheelAppliedVolts);
  }

  public void hoodRunVolts(double volts) {
    hoodIO.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void shooterRunVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    shooterVelocityFB.setGoal(velocityRadPerSec);

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stopShooter() {
    shooterIO.flywheelStop();
  }

  public void stopHood() {
    hoodIO.wristStop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getShooterVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(shooterInputs.flywheelVelocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getShooterCharacterizationVelocity() {
    return shooterInputs.flywheelVelocityRadPerSec;
  }

  public void setTargetShooterAngleRad(Rotation2d anglediff) {
    targetHoodAngleRad = anglediff.getRadians() * TURN_SHOOTER_RATIO;
  }
}

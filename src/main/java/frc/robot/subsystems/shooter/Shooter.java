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
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private ProfiledPIDController pid;
  private final SimpleMotorFeedforward ffModel;
  private static final double ENCODER_ANGLE_FIX = 1.5;
  // the ration for turning the shooter
  private static final double TURN_SHOOTER_RATIO = 5.4;
  private static double targetShooterAngleRad = 0.0;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: characterize real robot
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        pid =
            new ProfiledPIDController(
                1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.5, 99)); // FIXME: remove profile?
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        pid = new ProfiledPIDController(0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(0.5, 99));
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    //    io.setFlywheelVoltage(
    //        pid.calculate(inputs.flywheelVelocityRadPerSec)
    //            + ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
    targetShooterAngleRad = pid.getSetpoint().position * ENCODER_ANGLE_FIX;
    Logger.processInputs("Shooter", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setFlywheelVoltage(volts);
  }

  /**
   * Run open loop at the specified voltage.
   */
  public void runVoltage(Measure<Voltage> voltage) {
    runVolts(voltage.in(Volts));
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    pid.setGoal(velocityRadPerSec);

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.flywheelStop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.flywheelVelocityRadPerSec);
  }

  /**
   * Returns the current velocity in radians per second.
   */
  public Measure<Velocity<Angle>> getCharacterizationVelocity() {
    return RadiansPerSecond.of(inputs.flywheelVelocityRadPerSec);
  }

  /**
   * Returns the current velocity in radians per second.
   */
  public Measure<Angle> getCharacterizationPosition() {
    return Radians.of(inputs.flywheelPositionRad);
  }


  /**
   * Returns the current velocity in radians per second.
   */
  public Measure<Voltage> getCharacterizationVoltage() {
    return Volts.of(inputs.flywheelAppliedVolts);
  }

  /**
   * Returns the current velocity in radians per second.
   */
  public Measure<Current> getCharacterizationCurrent() {
    var sum = 0.0;
    for (double flywheelCurrentAmp : inputs.flywheelCurrentAmps) sum += flywheelCurrentAmp;

    sum = (inputs.flywheelCurrentAmps.length > 0) ? sum / inputs.flywheelCurrentAmps.length : 0.0;
    return Amps.of(sum);
  }

  public void setTargetShooterAngle(Rotation2d anglediff) {
    targetShooterAngleRad = anglediff.getRadians() * TURN_SHOOTER_RATIO;
  }
}

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import javax.swing.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterio;
  private final WristIO wristio;
  private final ShooterIOInputsAutoLogged shooterinputs = new ShooterIOInputsAutoLogged();
  private final WristIOInputsAutoLogged wristinputs = new WristIOInputsAutoLogged();
  private ProfiledPIDController shooterpid;
  private ProfiledPIDController wristpid;
  private final SimpleMotorFeedforward ffModel;
  private static final double ENCODER_ANGLE_FIX = 1.5;
  // the ration for turning the shooter
  private static final double TURN_SHOOTER_RATIO = 5.4;
  private static double targetShooterAngleRad = 0.0;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io, WristIO wristIO) {
    this.shooterio = io;
    this.wristio = wristIO;
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: characterize real robot
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        shooterpid =
            new ProfiledPIDController(
                1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.5, 99)); // FIXME: remove profile?
        wristpid = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        shooterpid =
            new ProfiledPIDController(0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(0.5, 99));
        wristpid = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 2));
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    shooterio.updateInputs(shooterinputs);
    wristio.updateInputs(wristinputs);
    shooterio.setFlywheelVoltage(
        shooterpid.calculate(shooterinputs.flywheelVelocityRadPerSec)
            + ffModel.calculate(
                shooterpid.getSetpoint().position, shooterpid.getSetpoint().velocity));
    targetShooterAngleRad = shooterpid.getSetpoint().position * ENCODER_ANGLE_FIX;
    Logger.processInputs("Shooter", shooterinputs);
    Logger.processInputs("Wrist", wristinputs);
  }

  /** Run open loop at the specified voltage. */
  public void shooterrunvolts(double volts) {
    shooterio.setFlywheelVoltage(volts);
  }

  public void wristrunvolts(double volts){
    wristio.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void shooterrunVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    shooterpid.setGoal(velocityRadPerSec);

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stopshooter() {
    shooterio.flywheelStop();
  }

  public void stopwrist(){
    wristio.wristStop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getShooterVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(shooterinputs.flywheelVelocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getShooterCharacterizationVelocity() {
    return shooterinputs.flywheelVelocityRadPerSec;
  }

  public void setTargetShooterAngleRad(Rotation2d anglediff) {
    targetShooterAngleRad = anglediff.getRadians() * TURN_SHOOTER_RATIO;
  }
}

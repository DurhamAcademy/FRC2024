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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.Seconds;

public class Module {
    private static final double WHEEL_RADIUS = 0.057;

  private LoggedDashboardNumber pPidRot = new LoggedDashboardNumber("Drive/Module/Rot P");
  private LoggedDashboardNumber dPidRot = new LoggedDashboardNumber("Drive/Module/Rot D");

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private double lastPositionMeters = 0.0; // Used for delta calculation

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        switch (index) {
          case 0:
            driveFeedforward = new SimpleMotorFeedforward(0.039527, 0.13437, 0.12428);
            driveFeedback = new PIDController(0.15254, 0.0, 0.0);
            break;
          case 1:
            driveFeedforward = new SimpleMotorFeedforward(0.024784, 0.13088, 0.1558/6.2831);
            driveFeedback = new PIDController(0.1295, 0.0, 0.0);
            break;
          case 2:
            driveFeedforward = new SimpleMotorFeedforward(0.077976, 0.13341, 0.13853/6.2831);
            driveFeedback = new PIDController(0.13535, 0.0, 0.0);
            break;
          case 3:
            driveFeedforward = new SimpleMotorFeedforward(0.077976, 0.12927, 0.1631/6.2831);
            driveFeedback = new PIDController(0.10222, 0.0, 0.0);
            break;
          default:
            driveFeedforward = new SimpleMotorFeedforward(.175, 0.127, .13);
            driveFeedback = new PIDController(0.15254, 0.0, 0.0);
            break;
        }
//        driveFeedback = new PIDController(0.0097924, 0.0, 0.0);//fixme: try commenting/uncommenting this line: it overrides the previous ones
        turnFeedback = new PIDController(5.0, 0.0, .02);
        break;
      case REPLAY:
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        turnFeedback = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        driveFeedback = new PIDController(0.1, 0.0, 0.0);
        turnFeedback = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        turnFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    dPidRot.setDefault(turnFeedback.getD());
    pPidRot.setDefault(turnFeedback.getP());

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  Measure<Velocity<Angle>> lastDriveVelocity = Units.RadiansPerSecond.zero();
  Measure<Time> lastTime = Units.Microsecond.of(Logger.getTimestamp()).minus(Seconds.of(0.02));

  public void periodic() {
    io.updateInputs(inputs);
    var time = Units.Microsecond.of(Logger.getTimestamp());
    var currentVelocity = Units.RadiansPerSecond.of(inputs.driveVelocityRadPerSec);
    var acceleration = (lastDriveVelocity.minus(currentVelocity)).per(lastTime.minus(time));
    var maxAcheivableAcceleration = Units.RadiansPerSecond.per(Seconds).of(driveFeedforward.maxAchievableAcceleration(inputs.driveAppliedVolts,lastDriveVelocity.in(Units.RadiansPerSecond)));
    var freeSpinningAmount = Math.min(acceleration.baseUnitMagnitude()/(maxAcheivableAcceleration.baseUnitMagnitude()), 1);
    Logger.recordOutput("Drive/Module" + index + "/Max Achievable Acceleration", maxAcheivableAcceleration);
    Logger.recordOutput("Drive/Module" + index + "/Acceleration", acceleration);
    Logger.recordOutput("Drive/Module" + index + "/Free Spinning Amount", freeSpinningAmount);
    lastDriveVelocity = currentVelocity;
    lastTime = time;
//    turnFeedback.setP(pPidRot.get());
//    turnFeedback.setD(dPidRot.get());
    //noinspection UnnecessaryCallToStringValueOf
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnVoltage(
              turnFeedback.calculate(inputs.turnAbsolutePosition.getRadians(), angleSetpoint.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
        double feedForwardValue = driveFeedforward.calculate(velocityRadPerSec);
        double feedBackValue = driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec);
        double sum = feedBackValue + feedForwardValue;
        io.setDriveVoltage(sum);
        Logger.recordOutput("Drive/Module "+index+"/Drive/FF Value", feedForwardValue);
        Logger.recordOutput("Drive/Module "+index+"/Drive/FB Value", feedBackValue);
        Logger.recordOutput("Drive/Module "+index+"/Drive/Voltage Sum", sum);
        Logger.recordOutput("Drive/Module "+index+"/Drive/feedback Setpoint", driveFeedback.getSetpoint());
        Logger.recordOutput("Drive/Module "+index+"/Drive/feedback Position Error", driveFeedback.getPositionError());
        Logger.recordOutput("Drive/Module "+index+"/Drive/feedback Velocity Error", driveFeedback.getVelocityError());

      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    Logger.recordOutput("Voltage for motor" + this.index, volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPosition.plus(turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module position delta since the last call to this method. */
  public SwerveModulePosition getPositionDelta() {
    var delta = new SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle());
    lastPositionMeters = getPositionMeters();
    return delta;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocityRadPerSec() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the drive velocity unitless. */
  public Measure<Angle> getCharacterizationDrivePosition() {
    return edu.wpi.first.units.Units.Radians.of(inputs.drivePositionRad);
  }
  /** Returns the turn velocity unitless. */
  public Measure<Angle> getCharacterizationTurnPosition() {
    return edu.wpi.first.units.Units.Radians.of(inputs.turnPosition.getRadians());
  }
  /** Returns the drive velocity unitless. */
  public Measure<Velocity<Angle>> getCharacterizationDriveVelocity() {
    return edu.wpi.first.units.Units.RadiansPerSecond.of(inputs.driveVelocityRadPerSec);
  }
  /** Returns the turn velocity unitless. */
  public Measure<Velocity<Angle>> getCharacterizationTurnVelocity() {
    return edu.wpi.first.units.Units.RadiansPerSecond.of(inputs.turnVelocityRadPerSec);
  }

  public int getIndex() {
    return index;
  }
}

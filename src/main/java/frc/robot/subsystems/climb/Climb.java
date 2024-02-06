package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final ProfiledPIDController pidController;
  double offset = 0.0;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        // FIXME: characterize real robot
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        pidController =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        pidController =
            new ProfiledPIDController(
                1.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(10, 10)); // fixme: tune velocity and acceleration
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        pidController =
            new ProfiledPIDController(0., 0., .0, new TrapezoidProfile.Constraints(0., 0.));
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setLeftVoltage(
      pidController.calculate(inputs.leftPositionRad)
        + ffModel.calculate(pidController.getSetpoint().velocity));
    io.setRightVoltage(
      pidController.calculate(inputs.rightPositionRad)
        + ffModel.calculate(pidController.getSetpoint().velocity));
    Logger.processInputs("Climb", inputs);
  }

  /** Runs left motor at the specified voltage. */
  public void runLeftVolts(double volts) {
    io.setLeftVoltage(volts);
  }

  /** Runs right motor at the specified voltage. */
  public void runRightVolts(double volts) {
    io.setRightVoltage(volts);
  }

  /** Run closed loop to the specified position. */
  public void runPosition(double position) {
    pidController.setGoal(position + offset);
    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", position);
  }

  public void resetPosition() {
    var oldGoal =
        new TrapezoidProfile.State(
            pidController.getGoal().position - offset, pidController.getGoal().velocity);
    offset = inputs.positionRad;
    pidController.setGoal(new TrapezoidProfile.State(oldGoal.position + offset, oldGoal.velocity));
  }

  /** Stops the left. */
  public void stop() {
    io.leftStop();
    io.rightStop();
  }

  /** Returns the current left motor angle of climb. */
  public leftRotation getAngle() {
    if (leftRelativeOffset == null) {
      return new leftRotation();
    } else {
      return inputs.leftPosition.plus(leftRelativeOffset);
    }

  /** Returns the current right angle of climb. */
  public rightRotation getAngle() {
    if (rightRelativeOffset == null) {
      return new rightRotation();
    } else {
      return inputs.rightPosition.plus(rightRelativeOffset);
    }

  /** Returns the current left position of the climb in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * LEFT_RADIUS;
  }
  /** Returns the current right position of the climb in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * RIGHT_RADIUS;
  }

  /** Returns the left motor velocity in radians/sec. */
  public double getLeftVelocityRadPerSec() {
    return inputs.leftVelocityRadPerSec;
    }

  /** Returns the current velocity in radians per second. */
  public double getRightVelocityRadPerSec() {
    return inputs.rightVelocityRadPerSec;
  }
}

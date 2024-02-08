package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static final double LEFT_RADIUS = Units.inchesToMeters(0.8); // FIGURE OUT RADIUS
  private static final double RIGHT_RADIUS = Units.inchesToMeters(0.8); // FIGURE OUT RADIUS
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final ProfiledPIDController pidController;
  private Rotation2d leftRelativeOffset = null; // Relative + Offset = Absolute
  private Rotation2d rightRelativeOffset = null; // Relative + Offset = Absolute

  double leftOffset = 0.0;
  double rightOffset = 0.0;
  double leftVelocity = 0.0;
  double rightVelocity = 0.0;
  double leftMeasuredPosition;
  double rightMeasuredPosition;
  double leftGoalPosition;
  double rightGoalPosition;

  public Climb(ClimbIO io) {
    this.io = io;
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
    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (leftRelativeOffset == null && inputs.leftPosition.getRadians() != 0.0) {
      leftRelativeOffset = inputs.leftPosition.minus(inputs.leftPosition);
    }
    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (rightRelativeOffset == null && inputs.rightPosition.getRadians() != 0.0) {
      rightRelativeOffset = inputs.rightPosition.minus(inputs.rightPosition);
    }
    io.updateInputs(inputs);
    // sets voltages
    io.setLeftVoltage(
        pidController.calculate(inputs.leftPositionRad, leftGoalPosition)
            + ffModel.calculate(pidController.getSetpoint().velocity));
    io.setRightVoltage(
        pidController.calculate(inputs.rightPositionRad, rightGoalPosition)
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

  /** Run closed loop to the specified position. (for left motor) */
  public void runLeftPosition(double leftPosition) {
    double leftGoalPosition = leftPosition;
    // Log climb setpoint
    Logger.recordOutput("Climb left motor/SetpointRPM", leftGoalPosition);
  }

  /** Run closed loop to the specified position. (for right motor) */
  public void runRightPosition(double rightPosition) {
    double rightGoalPosition = rightPosition;
    // Log climb setpoint
    Logger.recordOutput("Climb right motor/SetpointRPM", rightGoalPosition);
  }

    // idk what this does (for the left)
  public void resetLeftPosition() {
    leftOffset = inputs.leftPositionRad;
    pidController.reset(leftMeasuredPosition, leftVelocity = 0);
  }

  // idk what this does (for the right)
  public void resetRightPosition() {
    rightOffset = inputs.rightPositionRad;
    pidController.reset(rightMeasuredPosition, rightVelocity = 0);
  }

  /** Returns the current left motor angle of climb. */
  public Rotation2d getLeftAngle() {
    if (leftRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.leftPosition.plus(leftRelativeOffset);
    }
  }

  /** Returns the current right angle of climb. */
  public Rotation2d getRightAngle() {
    if (rightRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.rightPosition.plus(rightRelativeOffset);
    }
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

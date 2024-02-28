package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static final double LEFT_RADIUS = Units.inchesToMeters(0.8); // FIGURE OUT RADIUS
  private static final double RIGHT_RADIUS = Units.inchesToMeters(0.8); // FIGURE OUT RADIUS
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private Double leftRelativeOffset = null; // Relative + Offset = Absolute
  private Double rightRelativeOffset = null; // Relative + Offset = Absolute

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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // sets voltages
    //    io.setLeftVoltage(
    //        pidController.calculate(inputs.leftPositionRad, leftGoalPosition)
    //            + ffModel.calculate(pidController.getSetpoint().velocity));
    //    io.setRightVoltage(
    //        pidController.calculate(inputs.rightPositionRad, rightGoalPosition)
    //            + ffModel.calculate(pidController.getSetpoint().velocity));
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
    //    pidController.reset(inputs.leftPositionRad, leftVelocity = 0);
  }

  // idk what this does (for the right)
  public void resetRightPosition() {
    rightOffset = inputs.rightPositionRad;
    //    pidController.reset(inputs.rightPositionRad, rightVelocity = 0);
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

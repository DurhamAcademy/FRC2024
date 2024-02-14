package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ModeHelper {

  private final RobotContainer container;

  public ModeHelper(RobotContainer container) {
    this.container = container;
  }

  private Mode mode = Mode.NEUTRAL;

  private Command transition = null;

  public void switchTo(Mode target) {

    if (mode != Mode.NEUTRAL && target == mode) {
      switchTo(Mode.NEUTRAL);
      return;
    }

    Command exit = container.getExitCommand(mode);
    Command enter = container.getEnterCommand(target);

    transition =
        Commands.sequence(
            exit.withInterruptBehavior(
                Command.InterruptionBehavior.kCancelIncoming), /* command group to exit mode */
            enter);
    transition.schedule();
  }

  public void cancelTransition() {
    if (transition != null) transition.cancel();
  }
}

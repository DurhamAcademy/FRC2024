package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class SpecializedCommands {
    public static Command timeoutDuringAuto(double timeout) {
        return Commands.either(
                Commands.waitSeconds(timeout),
                Commands.none(),
                RobotState::isAutonomous
        );
    }

    public static Command timeoutDuringAutoSim(double timeout) {
        return Commands.either(
                Commands.waitSeconds(timeout),
                Commands.waitUntil(() -> false),
                () -> RobotState.isAutonomous() && Constants.currentMode.equals(Constants.Mode.SIM)
        );
    }
}

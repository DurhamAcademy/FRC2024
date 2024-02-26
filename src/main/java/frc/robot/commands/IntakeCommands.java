package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command intake(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromDegrees(0.0));
                    intake.setRollerPercentage(0.75);
                },
                intake);
    }

    public static Command idle(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromDegrees(-90.0));
                    intake.setRollerPercentage(0.0);
                },
                intake);
    }
}

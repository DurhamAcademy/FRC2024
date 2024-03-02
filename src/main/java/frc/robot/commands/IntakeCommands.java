package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command intakeCommand(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromDegrees(-5.0));
                    intake.setRollerPercentage(0.75);
                },
                intake);
    }

    public static Command flushIntake(Intake intake){
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromDegrees(-5.0));
                    intake.setRollerPercentage(-0.5);
                },
                intake);
    }

    public static Command idleCommand(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromRadians(-2.15));
                    intake.setRollerPercentage(0.0);
                },
                intake);
    }

}

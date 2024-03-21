package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class IntakeCommands {
    public static Command intakeCommand(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromDegrees(-10));
                    intake.setRollerVoltage(9.0);
                },
                intake);
    }

    public static Command safeIntakeCommand(Intake intake, Feeder feeder) {
        return intakeCommand(intake)
                .onlyWhile(() -> !feeder.getBeamBroken());
    }

    public static Command smartIntakeCommand(Intake intake, Feeder feeder) {
        return sequence(
                safeIntakeCommand(intake, feeder)
                        .until(feeder::getIntakeBeamBroken),
                intakeCommand(intake)
                        .withTimeout(0.2)
                        .onlyWhile(feeder::getIntakeBeamBroken),
                IntakeCommands.flushIntakeWithoutTheArmExtendedOutward(intake)
                        .onlyWhile(feeder::getIntakeBeamBroken)
        ).onlyIf(()-> !(feeder.getIntakeBeamBroken() || feeder.getBeamBroken()));
    }

    public static Command flushIntake(Intake intake){
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromRadians(-2.05));
                    intake.setRollerVoltage(-6.0);
                },
                intake);
    }

    public static Command flushIntakeWithoutTheArmExtendedOutward(Intake intake){
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromRadians(-2.05));
                    intake.setRollerVoltage(6.0);
                }, intake);
    }

    public static Command idleCommand(Intake intake) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromRadians(-2.15));
                    intake.setRollerVoltage(0.0);
                },
                intake);
    }

    public static Command setAngle(Intake intake, double angle) {
        return new RunCommand(
                () -> {
                    intake.setIntakePosition(Rotation2d.fromRadians(angle));
                    intake.setRollerVoltage(0.0);
                },
                intake);
    }

}

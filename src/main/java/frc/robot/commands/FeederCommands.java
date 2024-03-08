package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class FeederCommands {
    public static Command feedToShooter(Feeder feeder) {
        return sequence(
                run(() -> feeder.runVolts(12), feeder)
                        .onlyIf(feeder::getBeamBroken)
                        .until(() -> !feeder.getBeamBroken()),
                run(() -> feeder.runVolts(12), feeder)
                        .withTimeout(0.5)
        );
    }

    public static Command flushFeeder(Feeder feeder, Intake intake) {
        return new RunCommand(
                () -> {
                    feeder.runVolts(-4);
                    intake.setRollerVoltage(-4);
                },
                feeder, intake);
    }

    public static Command feedToBeamBreak(Feeder feeder, Intake intake) {
        return sequence(
                run(() -> {
                    feeder.runVolts(6);
                    intake.setRollerVoltage(6);
                }, feeder, intake)
                        .onlyWhile(() -> !feeder.getBeamBroken()),
                run(() -> {
                    feeder.runVolts(-4);
                    intake.setRollerVoltage(-4);
                }, feeder, intake)
                        .onlyWhile(feeder::getBeamBroken),
                run(() -> {
                    feeder.runVolts(3);
                    intake.setRollerVoltage(3);
                }, feeder, intake)
                        .onlyWhile(() -> !feeder.getBeamBroken())
        );
    }
}

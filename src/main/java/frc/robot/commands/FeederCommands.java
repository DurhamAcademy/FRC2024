package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.feeder.Feeder;

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

    public static Command flushFeeder(Feeder feeder){
        return new RunCommand(
                () -> {
                    feeder.runVolts(-0.75);
                },
                feeder);
    }

    public static Command feedToBeamBreak(Feeder feeder) {
        return sequence(
                run(() -> feeder.runVolts(8), feeder)
                        .onlyWhile(() -> !feeder.getBeamBroken()),
                run(() -> feeder.runVolts(-4), feeder)
                        .onlyWhile(feeder::getBeamBroken),
                run(() -> feeder.runVolts(3))
                        .onlyWhile(() -> !feeder.getBeamBroken())
        );
    }
}

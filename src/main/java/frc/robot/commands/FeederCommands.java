package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.feeder.Feeder;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class FeederCommands {
    public static Command idleFeeder(Feeder feeder) {
        return repeatingSequence(
                either(
                        run(() -> feeder.runVolts(0.0))
                                .onlyWhile(feeder::getBeamBroken),
                        sequence(
                                run(() -> feeder.runVolts(0.0))
                                        .onlyWhile(() -> !feeder.getBeamBroken()),
                                zeroToBeamBreak(feeder)
                        ),
                        feeder::getBeamBroken
                )
        );
    }
    public static Command feedToShooter(Feeder feeder) {
        return sequence(
                run(() -> feeder.runVolts(12), feeder)
                        .onlyIf(feeder::getBeamBroken)
                        .until(() -> !feeder.getBeamBroken()),
                run(() -> feeder.runVolts(12), feeder)
                        .withTimeout(0.5)
        );
    }

    public static Command flushFeeder(Feeder feeder) {
        return new RunCommand(
                () -> {
                    feeder.runVolts(-4);
                },
                feeder);
    }

    public static Command feedToBeamBreak(Feeder feeder) {
        return either(
                zeroToBeamBreak(feeder),
                sequence(
                        sequence(
                                run(() -> feeder.runVolts(6), feeder)
                                        .onlyWhile(() -> !feeder.getIntakeBeamBroken()),
                                run(() -> feeder.runVolts(6), feeder)
                                        .onlyWhile(feeder::getIntakeBeamBroken)
                        )
                                .onlyWhile(() -> !feeder.getBeamBroken()),
                        either(
                                zeroToBeamBreak(feeder),
                                slowToBeam(feeder),
                                feeder::getBeamBroken
                        )
                ),
                feeder::getBeamBroken);
    }

    private static Command slowToBeam(Feeder feeder) {
        return run(() -> feeder.runVolts(2), feeder)
                .onlyWhile(() -> !feeder.getBeamBroken());
    }

    private static Command zeroToBeamBreak(Feeder feeder) {
        return sequence(
                run(() -> feeder.runVolts(-6), feeder).onlyWhile(feeder::getBeamBroken),
                slowToBeam(feeder)
        );
    }

    public static Command humanPlayerIntake(Feeder feeder) {
        return run(() -> {
            feeder.runVolts(-2.0);
        });
    }

//    public static Command feedToBeamBreak(Feeder feeder) {
//        return sequence(
//                run(() -> {
//                    feeder.runVolts(6);
//                }, feeder)
//                        .onlyWhile(() -> !feeder.getBeamBroken()),
//                run(() -> {
//                    feeder.runVolts(-4);
//                }, feeder)
//                        .onlyWhile(feeder::getBeamBroken),
//                run(() -> {
//                    feeder.runVolts(3);
//                }, feeder)
//                        .onlyWhile(() -> !feeder.getBeamBroken())
//        );
//    }
}

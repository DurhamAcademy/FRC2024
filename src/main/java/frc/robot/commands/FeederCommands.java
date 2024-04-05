package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
                ).beforeStarting(()-> feeder.setState(Feeder.State.none))
        );
    }
    public static Command feedToShooter(Feeder feeder) {
        return sequence(
                run(() -> feeder.runVolts(9), feeder)
                        .onlyIf(feeder::getBeamBroken)
                        .raceWith(SpecializedCommands.timeoutDuringAutoSim(2))
                        .until(() -> !feeder.getBeamBroken()),
                run(() -> feeder.runVolts(9), feeder)
                        .withTimeout(0.5),
                runOnce(() -> feeder.runVolts(0.0))
        )
                .beforeStarting(()->feeder.setState(Feeder.State.feedingShooter))
                .finallyDo(interrupted -> feeder.setState(Feeder.State.none));
    }

    public static Command flushFeeder(Feeder feeder) {
        return startEnd(
                () -> feeder.runVolts(-4),
                () -> feeder.runVolts(0.0),
                feeder);
    }

    public static Command feedToBeamBreak(Feeder feeder) {
        return either(
                zeroToBeamBreak(feeder),
                sequence(
                        sequence(
                                run(() -> feeder.runVolts(6), feeder)
                                        .onlyWhile(() -> !feeder.getIntakeBeamBroken())
                                        .raceWith(SpecializedCommands.timeoutDuringAutoSim(2)),
                                run(() -> feeder.runVolts(6), feeder)
                                        .onlyWhile(feeder::getIntakeBeamBroken)
                                        .raceWith(SpecializedCommands.timeoutDuringAutoSim(3))
                        )
                                .onlyWhile(() -> !feeder.getBeamBroken()),
                        either(
                                zeroToBeamBreak(feeder),
                                slowToBeam(feeder),
                                feeder::getBeamBroken
                        )
                ),
                feeder::getBeamBroken)
                .beforeStarting(()-> feeder.setState(Feeder.State.zeroingNote))
                .finallyDo(interrupted -> feeder.setState(Feeder.State.none));

    }

    private static Command slowToBeam(Feeder feeder) {
        return run(() -> feeder.runVolts(2), feeder)
                .onlyWhile(() -> !feeder.getBeamBroken())
                .raceWith(SpecializedCommands.timeoutDuringAutoSim(2));
    }

    private static Command zeroToBeamBreak(Feeder feeder) {
        return sequence(
                run(() -> feeder.runVolts(-6), feeder).onlyWhile(feeder::getBeamBroken),
                slowToBeam(feeder)
        )
                .raceWith(SpecializedCommands.timeoutDuringAutoSim(1));
    }

    public static Command humanPlayerIntake(Feeder feeder) {
        return startEnd(
                () -> feeder.runVolts(-8.0),
                ()-> feeder.runVolts(0)
        );
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

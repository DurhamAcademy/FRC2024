package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.feeder.Feeder;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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

    public static Command flushFeeder(Feeder feeder) {
        return new RunCommand(
                () -> {
                    feeder.runVolts(-4);
                },
                feeder);
    }

    public static Command feedToBeamBreak(Feeder feeder) {
        return sequence(
                run(() -> {
                    feeder.runVolts(6);
                }, feeder)
                        .onlyWhile(() -> !feeder.getBeamBroken() && !feeder.getIntakeBeamBroken())
                        .onlyWhile(() -> !feeder.getBeamBroken()),
                print("Step 1"),
                run(() -> {
                    feeder.runVolts(6);
                }, feeder)
                        .onlyWhile(() -> !feeder.getBeamBroken() && feeder.getIntakeBeamBroken())
                        .onlyWhile(() -> !feeder.getBeamBroken())
                        .withTimeout(0.1),
                print("Step 2"),
                run(() -> {
                    feeder.runVolts(2);
                }, feeder)
                        .onlyWhile(() -> !feeder.getBeamBroken()),
                print("Step 3")
//,
//                run(() -> {
//                    feeder.runVolts(-4);
//                }, feeder)
//                        .onlyWhile(feeder::getBeamBroken),
//                run(() -> {
//                    feeder.runVolts(3);
//                }, feeder)
//                        .onlyWhile(() -> !feeder.getBeamBroken())
        );
    }

    public static Command humanPlayerIntake(Feeder feeder){
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

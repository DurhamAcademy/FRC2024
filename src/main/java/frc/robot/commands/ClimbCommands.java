package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.Climb;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.math.filter.Debouncer.DebounceType.kFalling;

public class ClimbCommands {
    // run left until cannot move, if cannot move
    public static Command zero(Climb climb, double currentThreshhold) {
        var debouncers = new Object() {
            public Debouncer leftDebounce = new Debouncer(1.0, kFalling);
            public boolean cannotMoveAL = false;
            public boolean cannotMoveAR = false;
            public boolean cannotMoveBL = false;
            public boolean cannotMoveBR = false;
            public Debouncer rightDebounce = new Debouncer(1.0, kFalling);

            public void reset() {
                leftDebounce = new Debouncer(1.0, kFalling);
                cannotMoveAL = false;
                cannotMoveAR = false;
                cannotMoveBL = false;
                cannotMoveBR = false;
                rightDebounce = new Debouncer(1.0, kFalling);
            }
        };
        return new InstantCommand(debouncers::reset).andThen(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                runLeft(climb, true)
                                        .onlyWhile(getDebounce(debouncers.leftDebounce, climb.getLeftVelocityRadPerSec(), () -> debouncers.cannotMoveAL = true))
                                        .withTimeout(1),
                                runLeft(climb, false).onlyIf(() -> !debouncers.cannotMoveAL)
                                        .onlyWhile(getDebounce(debouncers.leftDebounce, climb.getLeftVelocityRadPerSec(), () -> debouncers.cannotMoveBL = true))
                                        .withTimeout(5)
                        ),
                        new SequentialCommandGroup(
                                runRight(climb, true)
                                        .onlyWhile(getDebounce(debouncers.leftDebounce, climb.getRightVelocityRadPerSec(), () -> debouncers.cannotMoveAR = true))
                                        .withTimeout(1),
                                runRight(climb, false).onlyIf(() -> !debouncers.cannotMoveAR)
                                        .onlyWhile(getDebounce(debouncers.leftDebounce, climb.getRightVelocityRadPerSec(), () -> debouncers.cannotMoveBR = true))
                                        .withTimeout(5)
                        )
                )
        );
    }

    private static BooleanSupplier getDebounce(Debouncer debouncers, double climb, Runnable onTrue) {
        return () -> {
            boolean calculate = debouncers.calculate(Math.abs(climb) < 0.5);
            if (calculate) onTrue.run();
            return calculate;
        };
    }

    private static RunCommand runLeft(Climb climb, boolean direction) {
        return new RunCommand(() -> climb.runLeftVolts(direction ? 10 : -10));
    }

    private static RunCommand runRight(Climb climb, boolean direction) {
        return new RunCommand(() -> climb.runRightVolts(direction ? 10 : -10));
    }
}

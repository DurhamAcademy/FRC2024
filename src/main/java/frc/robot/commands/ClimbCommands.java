package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.climb.Climb;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.math.filter.Debouncer.DebounceType.kFalling;

public class ClimbCommands {
    // run left until cannot move, if cannot move
    public static Command zero(Climb climb, double currentThreshhold) {
        var debouncers = new Object() {
            public Debouncer leftDebounce = new Debouncer(1.0, kFalling);
            public boolean cannotMoveA = false;
            public boolean cannotMoveB = false;
            public Debouncer rightDebounce = new Debouncer(1.0, kFalling);
        };
        return new InstantCommand(() -> {
            debouncers.leftDebounce = new Debouncer(0.2, kFalling);
            debouncers.rightDebounce = new Debouncer(0.2, kFalling);
        }).andThen(
                runLeft(climb, true)
                        .onlyWhile(getDebounce(debouncers.leftDebounce, climb.getLeftVelocityRadPerSec(), () -> debouncers.cannotMoveA = true))
                        .withTimeout(1),
                runLeft(climb, false).onlyIf(() -> !debouncers.cannotMoveA)
                        .onlyWhile(getDebounce(debouncers.leftDebounce, climb.getLeftVelocityRadPerSec(), () -> debouncers.cannotMoveB = true))

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

    private static RunCommand runRight(Climb climb) {
        return new RunCommand(() -> climb.runRightVolts(direction ? 10 : -10));
    }
}

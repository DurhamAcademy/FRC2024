package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.filter.Debouncer.DebounceType.kFalling;
import static edu.wpi.first.wpilibj2.command.Commands.*;

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
                rightDebounce.calculate(true);
                leftDebounce.calculate(true);
            }
        };
        SequentialCommandGroup sequentialCommandGroup = new InstantCommand(debouncers::reset).andThen(
                parallel(
                        runLeft(climb, true)
                                .onlyWhile(getDebounce(debouncers.leftDebounce, climb::getLeftVelocityRadPerSec, () -> debouncers.cannotMoveAL = true))
                                .withTimeout(8),
                        runRight(climb, true)
                                .onlyWhile(getDebounce(debouncers.rightDebounce, climb::getRightVelocityRadPerSec, () -> debouncers.cannotMoveAR = true))
                                .withTimeout(8)
                )
        );
        sequentialCommandGroup.addRequirements(climb);
        return sequentialCommandGroup;
    }

    private static BooleanSupplier getDebounce(Debouncer debouncers, DoubleSupplier climb, Runnable onTrue) {
        return () -> {
            boolean calculate = debouncers.calculate(Math.abs(climb.getAsDouble()) > 0.5);
            if (calculate) onTrue.run();
            return calculate;
        };
    }

    public static Command zeroClimb(Climb climb){
        return run(() -> {
            climb.runLeftVolts(4.0);
            climb.runRightVolts(4.0);
        });
    }

    public static Command runClimb(Climb climb, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        return run(
                () -> {
                    climb.runLeftVolts(
                            MathUtil.applyDeadband(leftSupplier.getAsDouble(), 0.075) * 12);
                    climb.runRightVolts(
                            MathUtil.applyDeadband(rightSupplier.getAsDouble(), 0.075) * 12);
                },
                climb);
    }

    private static RunCommand runLeft(Climb climb, boolean direction) {
        return new RunCommand(() -> climb.runLeftVolts(direction ? 10 : -10));
    }

    private static RunCommand runRight(Climb climb, boolean direction) {
        return new RunCommand(() -> climb.runRightVolts(direction ? 10 : -10));
    }
}

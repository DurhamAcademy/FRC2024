package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.feeder.Feeder;

public class FeederCommands {
    public static Command feedToShooter(Feeder feeder) {
        return new RunCommand(
                () -> {
                    feeder.runVolts(8);
                },
                feeder)
                .onlyIf(() -> feeder.getBeamBroken())
                .until(() -> !feeder.getBeamBroken());
    }
}

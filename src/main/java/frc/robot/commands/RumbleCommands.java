package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControllerRumble;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static java.lang.Math.max;
import static java.lang.Math.pow;

public class RumbleCommands {
    public static Command rumbleLight(ControllerRumble controller) {
        return run(() -> controller.setRumbleLight(0.5), controller);
    }

    public static Command rumbleLightWithFalloff(ControllerRumble controller) {
        return RumbleCommands.rumbleLightWithFalloff(controller, 40.0);
    }

    public static Command rumbleLightWithFalloff(ControllerRumble controller, double falloffStrength) {
        var c = 0.025;
        var n = max(falloffStrength, 2);
        var offset = (pow(c, (-1) / (-n - 1)) * pow(1 / n, (-1) / (-n - 1)));
        var timer = new Timer();
        return run(() -> {
            var elapsed = timer.get();
            var strength = 1 / (pow(elapsed + offset, n) + 1);
            controller.setRumbleLight(strength);
        }, controller).beforeStarting(timer::restart);
    }

    public static Command noRumble(ControllerRumble controller) {
        return run(() -> controller.setRumbleLight(0.0), controller);
    }
}

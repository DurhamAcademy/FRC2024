package frc.robot.commands;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.lights.LEDs;

import java.util.Optional;

import static com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent.Percent30;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

public class LEDCommands {
    public static Command flameCommand(LEDs leds, double brightness) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0), 0);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, .5, false, candleLength), 1);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, .5, true, (candleLength) + stripLength), 2);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, .5, false, (candleLength) + stripLength * 2), 3);
                    candle.animate(new FireAnimation(brightness, 0, stripLength, 1, .5, true, (candleLength) + stripLength * 3), 4);
                },
                () -> {
                    for (int i = 0; i < 5; i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command flameCommand(LEDs leds) {
        return flameCommand(leds, 0.25);
    }

    public static Command disabled(LEDs leds, RobotContainer robotContainer) {
        var candle = leds.getCandle();

        return startEnd(
                () -> {
                    candle.animate(new RainbowAnimation(1, 0, stripLength * 4, false, candleLength), 1);
                },
                () -> {
                    for (int i = 0; i < 10; i++) candle.clearAnimation(i);
                },
                leds
        ).deadlineWith(run(()-> {
            boolean dsAttached = DriverStation.isDSAttached();
            boolean cameraConnected = robotContainer.drive.cameraConnected();
            boolean gyroConnected = robotContainer.drive.isGyroConnected();
            if (cameraConnected) {
                candle.clearAnimation(5);
                candle.setLEDs(0, 100, 0, 0, 1, 0);
            }
            else candle.animate(new StrobeAnimation(255, 0, 0, 0, 0, 1, 0), 5);
            Optional<Alliance> side = DriverStation.getAlliance();
            if (side.isEmpty())
                candle.animate(new StrobeAnimation(100,0,0, 0, 0, 1, 1), 7);//blinkred
            else if (side.get() == Alliance.Blue)
                candle.animate(new SingleFadeAnimation(0,0,100, 0, 0, 1, 1), 7);
            else if (side.get() == Alliance.Red) {

                candle.animate(new SingleFadeAnimation(100, 0, 0, 0, 0, 1, 1), 7);
            }
            else candle.animate(new StrobeAnimation(100,0,0, 0, 0, 1, 1), 7);//blinkred
            if (dsAttached) {
                candle.clearAnimation(8);
                candle.setLEDs(0, 100, 0, 0, 1, 3);
            }
            else candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 3), 8);
            if (gyroConnected) {
                candle.clearAnimation(9);
                candle.setLEDs(0, 100, 0, 0, 1, 4);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 5), 9);
            if (DriverStation.isFMSAttached()) {
                candle.clearAnimation(10);
                candle.setLEDs(0, 100, 0, 0, 1, 4);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 5), 10);
        }));
    }

    public static Command enabled(LEDs leds) {
        var candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0), 0);
                    candle.animate(new TwinkleAnimation(200, 200, 200, 1, 0, stripLength * 4, Percent30, candleLength), 1);
                },
                () -> {
                    for (int i = 0; i < 2; i++) candle.clearAnimation(i);
                },
                leds
        );
    }
}

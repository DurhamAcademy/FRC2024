package frc.robot.commands;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.LEDs;

import static com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent.Percent30;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

public class LEDCommands {
    public static Command flameCommand(LEDs leds, double brightness) {
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

    public static Command hasNoteCommand(LEDs leds){
        var candle = leds.getCandle();
        return runOnce(() -> candle.setLEDs(0, 255, 0));
    }
    public static Command shooterMaxVel(LEDs leds){
        var candle = leds.getCandle();
        return runOnce(() -> candle.animate(new StrobeAnimation(0, 255, 0 , 100, 1,8,0))); //when the flywheel at max velocity
    }
    public static Command shooterVel(LEDs leds){
        var candle = leds.getCandle();
        return runOnce(() -> candle.animate(new StrobeAnimation(255,255,0,100,1,8,0))); //when the flywheel is starting to spin to shoot, but not quite ready
    }








    public static Command flameCommand(LEDs leds) {
        return flameCommand(leds, 0.25);
    }

    public static Command disabled(LEDs leds) {
        var candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0), 0);
                    candle.animate(new RainbowAnimation(1, 0, stripLength * 4, false, candleLength), 1);
                },
                () -> {
                    for (int i = 0; i < 2; i++) candle.clearAnimation(i);
                },
                leds
        );
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

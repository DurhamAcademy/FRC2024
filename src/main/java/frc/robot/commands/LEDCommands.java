package frc.robot.commands;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.lights.LEDs;

import java.util.Optional;

import static com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent.Percent30;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.lights.LEDs.candleLength;
import static frc.robot.subsystems.lights.LEDs.stripLength;

public class LEDCommands {
    public static boolean wantsHPI;
    public static Command flameCommand(LEDs leds, double brightness) {
        if (leds == null) return none();
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

    public static Command setIntakeBoolean(){
        wantsHPI = !wantsHPI;
        return runOnce(() -> wantsHPI = !wantsHPI)
                .withName("Change Intake Mode");

    }

    public static Command setIntakeType(LEDs leds){
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return startEnd(
                () -> {
                    if(wantsHPI) {
                        candle.animate(new SingleFadeAnimation(255, 255, 255, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 255, 255, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 255, 255, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 255, 255, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 255, 255, 0, 0, stripLength * 4), candleLength);
                    }
                    else{
                        candle.animate(new SingleFadeAnimation(255, 127, 80, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 127, 80, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 127, 80, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 127, 80, 0, 0, stripLength * 4), candleLength);
                        candle.animate(new SingleFadeAnimation(255, 127, 80, 0, 0, stripLength * 4), candleLength);
                    }
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
                () -> candle.animate(new RainbowAnimation(1, 0, stripLength * 4, false, candleLength), 1),
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        ).deadlineWith(run(()-> {
            boolean dsAttached = DriverStation.isDSAttached();
            boolean cameraConnected = robotContainer.drive.cameraConnected();
            boolean gyroConnected = robotContainer.drive.isGyroConnected();
            if (cameraConnected) {
                candle.clearAnimation(2);
                candle.setLEDs(0, 100, 0, 0, 0, 1);
            } else candle.animate(new StrobeAnimation(255, 0, 0, 0, 0, 1, 0), 2);

            Optional<Alliance> side = DriverStation.getAlliance();
            if (side.isEmpty()) {
                candle.animate(new StrobeAnimation(100,100,100, 0, 0, 1, 1), 3);//blinkred
            } else if (side.get() == Alliance.Blue) {
                candle.animate(new SingleFadeAnimation(0,0,100, 0, 0, 1, 1), 3);
            } else if (side.get() == Alliance.Red) {
                candle.animate(new SingleFadeAnimation(100, 0, 0, 0, 0, 1, 1), 3);
            } else {
                candle.animate(new StrobeAnimation(100,0,0, 0, 0, 1, 1), 3);//blinkred
            }

            if (dsAttached) {
                candle.clearAnimation(4);
                candle.setLEDs(0, 100, 0, 0, 3, 1);
            }
            else candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 3), 4);

            if (gyroConnected) {
                candle.clearAnimation(5);
                candle.setLEDs(0, 100, 0, 0, 5, 1);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 5), 5);

            if (RobotController.getBatteryVoltage() > 13.0) {
                candle.clearAnimation(5);
                candle.setLEDs(0, 100, 0, 0, 6, 1);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1,6), 5);

            if (DriverStation.isFMSAttached()) {
                candle.clearAnimation(6);
                candle.setLEDs(0, 100, 0, 0, 7, 1);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 7), 6);
        })).handleInterrupt(() -> {
            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                candle.clearAnimation(i);
            }
        });
    }

    public static Command enabled(LEDs leds) {
        var candle = leds.getCandle();
        return startEnd(
                () -> {
                    candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0), 0);
                    candle.animate(new TwinkleAnimation(200, 200, 200, 1, 0, stripLength * 4, Percent30, candleLength), 1);
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }
}

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
        return runOnce(() -> wantsHPI = !wantsHPI)
                .withName("Change Intake Mode");

    }

    public static Command setIntakeType(LEDs leds){
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return runEnd(
                () -> {
                    if(wantsHPI) {
                        candle.animate(new SingleFadeAnimation(255, 255, 50, 0, .1, stripLength * 4, candleLength), 0);
                    }
                    else{
                        candle.animate(new SingleFadeAnimation(0, 0, 80, 0, .1, stripLength * 4, candleLength), 0);
                    }
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command ledsUp(LEDs leds){
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return runEnd(
                () -> {
                    candle.setLEDs(0, 0, 0, 0, candleLength, stripLength/2); // up half 1
                    candle.setLEDs(100, 100, 100, 0, candleLength + stripLength*1 - stripLength/2, stripLength); // up half 2 down half 2
                    candle.setLEDs(0, 0, 0, 0, candleLength + stripLength*2 - stripLength/2, stripLength); // down half 1 up half 1
                    candle.setLEDs(100, 100, 100, 0, candleLength + stripLength*3 - stripLength/2, stripLength); // up half 2 down half 2
                    candle.setLEDs(0, 0, 0, 0, candleLength + stripLength*4 - stripLength/2, stripLength/2); // down half 1

                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command ledsDown(LEDs leds){
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return runEnd(
                () -> {
                    candle.setLEDs(100, 100, 100, 0, candleLength, stripLength/2); // up half 1
                    candle.setLEDs(0, 0, 0, 0, candleLength + stripLength*1 - stripLength/2, stripLength); // up half 2 down half 2
                    candle.setLEDs(100, 100, 100, 0, candleLength + stripLength*2 - stripLength/2, stripLength); // down half 1 up half 1
                    candle.setLEDs(0, 0, 0, 0, candleLength + stripLength*3 - stripLength/2, stripLength); // up half 2 down half 2
                    candle.setLEDs(100, 100, 100, 0, candleLength + stripLength*4 - stripLength/2, stripLength/2); // down half 1

                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command dropNoteEmily(LEDs leds){
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return startEnd(
                () -> {
                    candle.animate(new LarsonAnimation(255, 255, 50, 0, 0, stripLength, LarsonAnimation.BounceMode.Back, stripLength, candleLength), 0);
                    candle.animate(new LarsonAnimation(255, 255, 50, 0, 0, stripLength, LarsonAnimation.BounceMode.Front, stripLength, candleLength + stripLength), 1);
                    candle.animate(new LarsonAnimation(255, 255, 50, 0, 0, stripLength, LarsonAnimation.BounceMode.Back, stripLength, candleLength + stripLength*2), 2);
                    candle.animate(new LarsonAnimation(255, 255, 50, 0, 0, stripLength, LarsonAnimation.BounceMode.Front, stripLength, candleLength + stripLength*3), 3);
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command hasNote(LEDs leds){
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return startEnd(
                () -> {
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Back, stripLength, candleLength), 0);
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Front, stripLength, candleLength + stripLength), 1);
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Back, stripLength, candleLength + stripLength*2), 2);
                    candle.animate(new LarsonAnimation(255, 165, 0, 0, 0, stripLength, LarsonAnimation.BounceMode.Front, stripLength, candleLength + stripLength*3), 3);
                },
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        );
    }

    public static Command flameCommand(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        return flameCommand(leds, 0.25);
    }

    public static Command disabled(LEDs leds, RobotContainer robotContainer) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
        var candle = leds.getCandle();

        return startEnd(
                () -> candle.animate(new RainbowAnimation(1, 0, stripLength * 4, false, candleLength), 1),
                () -> {
                    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) candle.clearAnimation(i);
                },
                leds
        ).deadlineWith(run(()-> {
            boolean dsAttached = DriverStation.isDSAttached();
            boolean cameraConnected = robotContainer.drive.cameraCount() > 1;
            boolean gyroConnected = robotContainer.drive.isGyroConnected();
            if (cameraConnected) {
                candle.clearAnimation(2);
                candle.setLEDs(0, 100, 0, 0, 0, 1);
            } else candle.animate(new StrobeAnimation(255, 0, 0, 0, 0, 1, 0), 2);

            Optional<Alliance> side = DriverStation.getAlliance();
            if (side.isEmpty()) {
                candle.animate(new StrobeAnimation(50,50,50, 0, 0, 1, 1), 3);//blinkred
            } else if (side.get() == Alliance.Blue) {
                candle.animate(new SingleFadeAnimation(0,0,100, 0, 0, 1, 1), 3);
            } else if (side.get() == Alliance.Red) {
                candle.animate(new SingleFadeAnimation(100, 0, 0, 0, 0, 1, 1), 3);
            } else {
                candle.animate(new StrobeAnimation(100,0,0, 0, 0, 1, 1), 3);//blinkred
            }

            if (dsAttached) {
                candle.clearAnimation(4);
                candle.setLEDs(0, 100, 0, 0, 2, 1);
            }
            else candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 2), 4);

            if (gyroConnected) {
                candle.clearAnimation(5);
                candle.setLEDs(0, 100, 0, 0, 3, 1);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 3), 5);

            if (RobotController.getBatteryVoltage() > 13.0) {
                candle.clearAnimation(5);
                candle.setLEDs(0, 100, 0, 0, 4, 1);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1,4), 5);

            if (DriverStation.isFMSAttached()) {
                candle.clearAnimation(6);
                candle.setLEDs(0, 100, 0, 0, 5, 1);
            }
            else
                candle.animate(new StrobeAnimation(100, 0, 0, 0, 0, 1, 5), 6);
        })).handleInterrupt(() -> {
            for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
                candle.clearAnimation(i);
            }
        });
    }

    public static Command enabled(LEDs leds) {
        if (leds == null) return none();
        if (leds.getCandle() == null) return idle(leds);
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

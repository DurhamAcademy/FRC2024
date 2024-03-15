package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDs extends SubsystemBase {
    public static final int stripLength = 16;
    public static final int stripCount = 4;
    public static final int candleLength = 8; // 0-7
    CANdle candle;
    int lastTimeMod = -1;
    public LEDs() {
        if (RobotBase.isReal()) {
            candle = new CANdle(0);
            candle.configFactoryDefault();
            CANdleConfiguration config = new CANdleConfiguration();
            config.disableWhenLOS = false;
            config.statusLedOffWhenActive = true;
            config.stripType = CANdle.LEDStripType.GRB;
            config.brightnessScalar = 1.0;
            config.v5Enabled = false;
            config.enableOptimizations = true;
            candle.configAllSettings(config);
            candle.animate(new RgbFadeAnimation(1.0, 0.5, candleLength, 0), 0);
            candle.animate(new FireAnimation(1, 0.0001, stripLength, 1, .5, !true, candleLength), 1);
            candle.animate(new FireAnimation(1, 0.0001, stripLength, 1, .5, !false, (candleLength) + stripLength), 2);
            candle.animate(new FireAnimation(1, 0.0001, stripLength, 1, .5, !true, (candleLength) + stripLength * 2), 3);
            candle.animate(new FireAnimation(1, 0.0001, stripLength, 1, .5, !false, (candleLength) + stripLength * 3), 4);
        }
    }

    @Override
    public void periodic() {
//        if (RobotBase.isReal()) {
//            int timeMod = (int) ((RobotController.getFPGATime() / 1_000_000.0) % 60 * 10);
//            if (lastTimeMod != timeMod) {
//                lastTimeMod = timeMod;
//                for (int i = 0; i < Math.max(candle.getMaxSimultaneousAnimationCount(), 6); i++) {
//                    candle.clearAnimation(i);
//                }
//                switch (timeMod) {
//                    case 0:
//                        candle.animate(new TwinkleAnimation(100, 100, 100, 0, .5, stripLength * 4, TwinkleAnimation.TwinklePercent.Percent64, candleLength - 1));
//                        break;
//                    case 2:
//                        candle.animate(new ColorFlowAnimation(255, 255, 255, 0, .5, candleLength, ColorFlowAnimation.Direction.Forward), 0);
//                        candle.animate(new ColorFlowAnimation(255, 255, 255, 0, .5, stripLength, ColorFlowAnimation.Direction.Forward, (candleLength - 1)), 1);
//                        candle.animate(new ColorFlowAnimation(100, 255, 100, 0, .5, stripLength, ColorFlowAnimation.Direction.Forward, (candleLength - 1) + stripLength), 2);
//                        candle.animate(new ColorFlowAnimation(255, 100, 255, 0, .5, stripLength, ColorFlowAnimation.Direction.Forward, (candleLength - 1) + stripLength * 2), 3);
//                        candle.animate(new ColorFlowAnimation(255, 000, 100, 0, .5, stripLength, ColorFlowAnimation.Direction.Forward, (candleLength - 1) + stripLength * 3), 4);
//                        break;
//                    case 3:
//                        candle.animate(new LarsonAnimation(255, 255, 255, 0, .5, 76, LarsonAnimation.BounceMode.Center, 6, candleLength - 1));
//                        break;
//                    case 4:
//                        candle.animate(new RainbowAnimation(1, .5, stripLength * 4));
//                        break;
//                    case 5:
//                        candle.animate(new RgbFadeAnimation(1, .5, stripLength * 4));
//                        break;
//                    case 6:
//                        candle.animate(new TwinkleOffAnimation(100, 100, 100, 0, .5, 76, TwinkleOffAnimation.TwinkleOffPercent.Percent30));
//                        break;
//                    case 1:
//                    default:
//        candle.animate(new StrobeAnimation(255, 255, 255, 0, 0.5, candleLength), 0);
//        candle.animate(new FireAnimation(1, 1, stripLength, 0.5, 1, false, candleLength - 1), 0);
//        candle.animate(new FireAnimation(1, 1, stripLength, 0.5, 1, true, (candleLength - 1)), 1);
//        candle.animate(new FireAnimation(1, 1, stripLength, 0.5, 1, false, (candleLength - 1) + stripLength), 2);
//        candle.animate(new FireAnimation(1, 1, stripLength, 0.5, 1, true, (candleLength - 1) + stripLength * 2), 3);
//        break;
//                }
//            }
//        }
    }

    public CANdle getCandle() {
        return candle;
    }
}

package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;


public class LEDs extends SubsystemBase {

    Shooter shooter;
    public static final int stripLength = 16;
    public static final int stripCount = 4;

    Trigger shootingReady = new Trigger(shooter::allAtSetpoint);

    Trigger noNote;
    Trigger yesNote;


    public static final int candleLength = 8; // 0-7
    CANdle candle;
    public LEDs() {
        if (RobotBase.isReal()) {
            candle = new CANdle(0);
            candle.configFactoryDefault();
            CANdleConfiguration config = new CANdleConfiguration();
            config.disableWhenLOS = false;
            config.statusLedOffWhenActive = true;
            config.stripType = CANdle.LEDStripType.RGB;
            config.brightnessScalar = 1.0;
            config.v5Enabled = false;
            config.enableOptimizations = true;
            candle.configAllSettings(config);
        }
    }
    public void robotInit(){
        candle.setLEDs(255,255,255); //sets the LEDs to white
    }

    Runnable shooting = () ->
    {

    };
    public void red(){candle.setLEDs(255,0,0);}
    public void blue(){ candle.setLEDs(0,0,255);}
    public void green(){candle.setLEDs(0,255,0);}
    public void yellow(){candle.setLEDs(255,255,0);}







    @Override
    public void periodic() {

    }

    public CANdle getCandle() {
        return candle;
    }
}

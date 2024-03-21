package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDCommands;


public class LEDs extends SubsystemBase {
    public static final int stripLength = 16;
    public static final int stripCount = 4;
    Trigger beamBroken;
    Trigger maxVel;
    Trigger startShooter;
    LEDs led;









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
    public void Init(){
        candle.setLEDs(255,255,255);//sets it to white
    }
    @Override
    public void periodic() {
        beamBroken.onTrue(LEDCommands.hasNoteCommand(led));
        maxVel
                .and(beamBroken).onTrue(LEDCommands.shooterMaxVel(led));
        startShooter.onTrue(LEDCommands.shooterVel(led));
        startShooter.and(beamBroken).onTrue(LEDCommands.flameCommand(led));
        startShooter.and(beamBroken.negate()).onTrue(
                run(() -> candle.setLEDs(255,0,0))

        );
    }
    public CANdle getCandle() {
        return candle;
    }
}

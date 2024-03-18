package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class HoodIOSparkMax implements HoodIO {
    private static final double GEAR_RATIO = 1.5;
    private static final double MOTOR_TO_ROBOT = (1 / (36 * GEAR_RATIO)) * Math.PI * 2;

    private final CANSparkMax leader = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);

    private final SparkAbsoluteEncoder encoder =
            leader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);

    private final RelativeEncoder motorEncoder = leader.getEncoder();
    boolean hasReset = false;
    DigitalInput hoodLimitSwitch = new DigitalInput(5);

    public HoodIOSparkMax() {
        REVLibError[] codes = new REVLibError[11];
        for (int tries = 0; tries < 5; tries++) {
            int a = 0;
            codes[a++] = leader.restoreFactoryDefaults();
            codes[a++] = leader.setCANTimeout(250);
            leader.setInverted(false);
            codes[a++] = leader.enableVoltageCompensation(12.0);
            codes[a++] = leader.setSmartCurrentLimit(50);

            codes[a++] = motorEncoder.setPositionConversionFactor(MOTOR_TO_ROBOT);
            codes[a++] = motorEncoder.setVelocityConversionFactor(MOTOR_TO_ROBOT);
            codes[a++] = motorEncoder.setPosition(encoder.getPosition());
            codes[a++] = encoder.setInverted(true);

            codes[a++] = leader.burnFlash();

            codes[a++] = motorEncoder.setPositionConversionFactor(MOTOR_TO_ROBOT);
            codes[a++] = motorEncoder.setVelocityConversionFactor(MOTOR_TO_ROBOT);
//            codes[a] = motorEncoder.setPosition(MathUtil.angleModulus(encoder.getPosition() * Math.PI * 2) / GEAR_RATIO);
            boolean failed = false;
            for (REVLibError code : codes) {
                if (code != REVLibError.kOk) {
                    failed = true;
                    System.out.println("an error occured while starting the motor");
                    System.out.println(Arrays.deepToString(codes));
                    System.out.println("waiting 1 second");
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        System.out.println("an error occured while sleeping:");
                        System.out.println(e);
                    }
                }
            }
            if (!failed) break;
        }
        System.out.println("printing codes:");
        System.out.println(Arrays.deepToString(codes));
        System.out.println("finished printing codes.");
    }

    public void updateInputs(HoodIOInputs inputs) {
        inputs.isStalled = leader.getFault(CANSparkBase.FaultID.kStall);
        inputs.hoodPositionRad = (absoluteEncoder.getAbsolutePosition() * Math.PI * 2) / GEAR_RATIO;
        inputs.motorPositionRad = motorEncoder.getPosition();
        inputs.hoodAppliedVolts = leader.getBusVoltage() * leader.getAppliedOutput();
        inputs.hoodCurrentAmps = new double[]{leader.getOutputCurrent()};
        inputs.hoodVelocityRadPerSec = (motorEncoder.getVelocity() * Math.PI * 2) / GEAR_RATIO;
        inputs.hoodTemperature = new double[]{leader.getMotorTemperature()};
        inputs.islimitSwitchPressed = hoodLimitSwitch.get();
    }

    public void setBrakeMode(boolean enable) {
        leader.setIdleMode(enable ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void setVoltage(double volts) {
        Logger.recordOutput("HoodVoltage", volts);
        leader.setVoltage(MathUtil.clamp(volts, -5.0, 5.0));
    }
}

package frc.robot.subsystems.shooter.test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class Test extends SubsystemBase {
    private final TestIO io;
    private final TestIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    public Test(ShooterIO io) {

    }

    public void periodic() {

    }
    /** Run open loop at the specified voltage. */
    public void runVolts(double volts) {
        io.setFlywheelVoltage(volts);
    }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    private final MechanismLigament2d ligament1;
    private final MechanismLigament2d ligament1A;
    private final MechanismLigament2d ligament2;
    private final MechanismLigament2d ligament2A;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    ProfiledPIDController armFB;
    ArmFeedforward armFF;
    IntakeIO io;
    Mechanism2d mechanism2d = new Mechanism2d(0, 0);

    // double position = 0.0;
    Rotation2d armTarget = Rotation2d.fromDegrees(-90);

    Rotation2d off1 = new Rotation2d(1.2466477);
    Rotation2d off1A = new Rotation2d(0.313352305);
    Rotation2d off2 = new Rotation2d(1.36216528);
    Rotation2d off2A = new Rotation2d(0.154247715);
    Rotation2d quarterTurn = Rotation2d.fromRadians(Math.PI / 2);

    public Intake(IntakeIO io) {
        this.io = io;
        armFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
        armFB = new ProfiledPIDController(7.0, 0.0, 0.0, new Constraints(RadiansPerSecond.of(18), RadiansPerSecond.per(Second).of(240)));
        var root = mechanism2d.getRoot("Root", .305, .220);
        armFB.enableContinuousInput(-Math.PI, Math.PI);
        armFB.setTolerance(0.025);
        ligament1 = new MechanismLigament2d("Intake", .135, off1.getDegrees(), .1, new Color8Bit(1, 1, 1));
        ligament1A = new MechanismLigament2d("Intake", 0.232427, off1A.minus(quarterTurn).getDegrees(), .5, new Color8Bit(1, 1, 1));
        ligament2 = new MechanismLigament2d("Intake2", .227, off2.getDegrees(), .1, new Color8Bit(1, 1, 1));
        ligament2A = new MechanismLigament2d("Intake2", 0.232983, off2A.minus(quarterTurn).getDegrees(), .5, new Color8Bit(1, 1, 1));
        root.append(ligament1).append(ligament1A);
        root.append(ligament2).append(ligament2A);

        SmartDashboard.putNumber("IntakePIDD", armFB.getD());
        SmartDashboard.putNumber("IntakePIDP", armFB.getP());
    }


    boolean mustReset = true;
    private double rollerVoltageSetpoint = 0.0;

    public void resetArmFB() {
        armFB.reset(new TrapezoidProfile.State(Radians.of(inputs.armPositionRad), RadiansPerSecond.of(inputs.armVelocityRadPerSec)));
    }

    public void setIntakePosition(Rotation2d position) {
        armTarget = position;
    }

    @AutoLogOutput
    public boolean feedControlDisabled() {
        return disableFeedControl;
    }

    public void setFeedControl(boolean disabled) {
        this.disableFeedControl = disabled;
    }

    boolean disableFeedControl = false;
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        if(this.getCurrentCommand() != null) {
            Logger.recordOutput("Commands/Intake", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Commands/Intake", "");
        }
        if(inputs.armPositionRad < 1.715 && inputs.armPositionRad > -0.97){
            io.setRollerPercent(0.0);
        }
        if (mustReset) {
            resetArmFB();
            mustReset = false;
        }
        if (disableFeedControl) {

        } else if (RobotController.isSysActive())
            io.setArmVoltage(
                    armFB.calculate(inputs.armPositionRad, MathUtil.angleModulus(armTarget.getRadians()))
                            + armFF.calculate(armFB.getSetpoint().position, armFB.getSetpoint().velocity));
        else io.setArmVoltage(0.0);

        Rotation2d rotation2d = new Rotation2d(inputs.armPositionRad);
        ligament1.setAngle(
                rotation2d
                        .plus(off1)
                        .minus(quarterTurn)
                        .times(-1));
        ligament2.setAngle(
                rotation2d
                        .plus(off2)
                        .minus(quarterTurn)
                        .times(-1));
        Logger.recordOutput("Intake", mechanism2d);

        io.setRollerVoltage(rollerVoltageSetpoint/* * ((inputs.armPositionRad > -4.5) ? -1 : 1)*/);
    }

    public void setRollerVoltage(double voltage) {
        rollerVoltageSetpoint = voltage;
    }

    public Measure<Voltage> getArmCharacterizationVoltage() {
        return Volts.of(inputs.armAppliedVolts);
    }

    public Measure<Angle> getArmCharacterizationPosition() {
        return Radians.of(inputs.armPositionRad);
    }

    public Measure<Velocity<Angle>> getArmCharacterizationVelocity() {
        return RadiansPerSecond.of(inputs.armVelocityRadPerSec);
    }

    /**
     * Run open loop at the specified voltage.
     */
    public Measure<Current> getArmCharacterizationCurrent() {
        double sum = 0.0;
        for (int i = inputs.rollerCurrentAmps.length - 1; i >= 0; i--) {
            sum += inputs.rollerCurrentAmps[i];
        }
        if (inputs.rollerCurrentAmps.length != 0) {
            return Amps.of(sum / inputs.rollerCurrentAmps.length);
        } else return Amps.zero();
    }

    public void runArmVolts(Measure<Voltage> voltageMeasure) {
        io.setArmVoltage(voltageMeasure.in(Volts));
    }
}

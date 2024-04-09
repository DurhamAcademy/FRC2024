// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectoryState;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.*;
import frc.robot.subsystems.ControllerRumble;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.lights.LEDs;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Mode;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.beans.FeatureDescriptor;

import static edu.wpi.first.units.BaseUnits.Voltage;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.commands.FeederCommands.feedToBeamBreak;
import static frc.robot.commands.FeederCommands.feedToShooter;
import static frc.robot.commands.IntakeCommands.*;
import static frc.robot.commands.RumbleCommands.*;
import static frc.robot.commands.ShooterCommands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static SysIDMode sysIDMode = SysIDMode.Disabled;
    public final Shooter shooter;
    public final Feeder feeder;
    public final Intake intake;
    public final Climb climb;
    //    private final Dashboard dashboard;
    public final LEDs leds;
    private final ControllerRumble driverRumble = new ControllerRumble(0);
    private final ControllerRumble operatorRumble = new ControllerRumble(1);

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<SmartCommandsMode> smartCommandsMode =
            new LoggedDashboardChooser<>("Use Smart Commands", new SendableChooser<>());
    private final LoggedDashboardNumber flywheelSpeedInput =
            new LoggedDashboardNumber("Flywheel Speed", 1500.0);
    private final ReactionObject reactions;
    LoggedDashboardNumber angleOffsetInput = new LoggedDashboardNumber("Angle Offset", 0.0);

    // Subsystems
    public Drive drive;
    LoggedDashboardBoolean invertX = new LoggedDashboardBoolean("Invert X Axis", false);
    LoggedDashboardBoolean invertY = new LoggedDashboardBoolean("Invert Y Axis", false);
    LoggedDashboardBoolean invertOmega = new LoggedDashboardBoolean("Invert Omega Axis", false);


    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        leds = new LEDs();
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOSparkMax(0){},
                                new ModuleIOSparkMax(1){},
                                new ModuleIOSparkMax(2){},
                                new ModuleIOSparkMax(3){},
                                new VisionIOReal[]{
                                        new VisionIOReal("ShootSideCamera"),
                                        new VisionIOReal("RightCamera")
                                },
                                new LimelightNoteDetection());
                shooter = new Shooter(new ShooterIOTalonFX(), new HoodIOSparkMax());
                feeder = new Feeder(new FeederIOTalonFX());
                intake = new Intake(new IntakeIOSparkMax());
                climb = new Climb(new ClimbIOSparkMax());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new VisionIOSim(
                                        "ShootSideCamera", () -> (drive == null) ? (drive.getPose()) : new Pose2d()),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new LimelightNoteDetection() {});
                shooter = new Shooter(new ShooterIOSim(), new HoodIO() {});
                feeder = new Feeder(new FeederIOSim());
                intake = new Intake(new IntakeIOSim());
                climb = new Climb(new ClimbIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new VisionIO[]{new VisionIO() {
                                    @Override
                                    public String getCameraName() {
                                        return "ShootSideCamera";
                                    }}, new VisionIO() {
                                    public String getCameraName() {return "RightCamera";}
                                }},
                                new LimelightNoteDetection(){});
                shooter = new Shooter(new ShooterIO() {
                }, new HoodIO() {
                });
                feeder = new Feeder(new FeederIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                climb = new Climb(new ClimbIO() {
                });
                break;
        }

        var command =
                DriveCommands.aimAtSpeakerCommand(
                        drive,
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX);

        NamedCommands.registerCommand(
                "Aim Drivetrain",
                command.getCommand()
        );
        NamedCommands.registerCommand(
                "Ready Shooter",
                autoAim(shooter, drive, feeder).asProxy()
        );
        NamedCommands.registerCommand(
                "Zero Feeder",
                FeederCommands.feedToBeamBreak(feeder).asProxy()
        );
        NamedCommands.registerCommand(
                "Zero Hood",
                ShooterCommands.simpleHoodZero(shooter).asProxy()
        );
        NamedCommands.registerCommand(
                "Auto Point",
                ShooterCommands.autoAim(shooter, drive, feeder).asProxy()
        );
        NamedCommands.registerCommand(
                "Shoot When Ready",
                sequence(
                        waitUntil(feeder::getBeamBroken),
                        sequence(
                                waitUntil(() -> (shooter.allAtSetpoint() && (shooter.getShooterVelocityRPM() > 1000))),
                                feedToShooter(feeder)
                        )
                                .onlyWhile(() -> !feeder.getBeamBroken())
                                .withTimeout(3.0)
                ).raceWith(SpecializedCommands.timeoutDuringAutoSim(3.0))
        );
        NamedCommands.registerCommand(
                "Shoot",
                sequence(
                        feedToBeamBreak(feeder),
                        waitUntil(() -> (shooter.allAtSetpoint() && (shooter.getShooterVelocityRPM() > 1000))),
                        feedToShooter(feeder)
                )
                        .deadlineWith(ShooterCommands.JustShoot(shooter))
                        .withTimeout(4.0).asProxy());
        NamedCommands.registerCommand(
                "Intake Note",
                smartIntakeCommand(intake, feeder).raceWith(FeederCommands.feedToBeamBreak(feeder))
                        .andThen(either(
                                none(),
                                race(
                                        feedToBeamBreak(feeder).withTimeout(5),
                                        flushIntakeWithoutTheArmExtendedOutward(intake, feeder)
                                ),
                                feeder::getBeamBroken
                        )).withTimeout(3.0).asProxy()
        );
        NamedCommands.registerCommand(
                "Intake",
                intakeCommand(intake)
                        .withTimeout(1.0)
        );
        NamedCommands.registerCommand(
                "Force Shoot",
                sequence(
                ShooterCommands.forceShoot(shooter),
                        waitUntil(() -> (shooter.allAtSetpoint() && (shooter.getShooterVelocityRPM() > 1000))),
                                run(()->{feeder.runVolts(8);}, feeder)
                ).asProxy()
        );
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
//        dashboard = new Dashboard(autoChooser, drive, shooter, feeder, intake, vision, this.smartCommandsMode);
        this.reactions = new ReactionObject(
                new Trigger(feeder::getIntakeBeamBroken),
                new Trigger(feeder::getBeamBroken),
                new Trigger(shooter::allAtSetpoint),
                new Trigger(shooter::hoodAtSetpoint),
                new Trigger(shooter::flywheelAtSetpoint),
                new Trigger(RobotState::isTeleop),
                new Trigger(RobotState::isAutonomous),
                new Trigger(RobotState::isEnabled)
        );

        configureButtonBindings();
        configureReactions();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        switch (sysIDMode) {
            case Disabled:
                // ---- DEFAULT COMMANDS ----
                drive.setDefaultCommand(
                        DriveCommands.joystickDrive(
                                drive,
                                () -> (-driverController.getLeftY() * (invertX.get()?-1:1)),
                                () -> (-driverController.getLeftX() * (invertY.get()?-1:1)),
                                () -> (-driverController.getRightX()) * (invertOmega.get()?-1:1)));
                intake.setDefaultCommand(IntakeCommands.idleCommand(intake));
                feeder.setDefaultCommand(FeederCommands.idleFeeder(feeder));
                shooter.setDefaultCommand(
                        either(
                                ShooterCommands.shooterIdle(shooter),
                                sequence(
                                        ShooterCommands.shooterIdle(shooter).until(shooter::hoodAtSetpoint).withTimeout(.5),
                                        ShooterCommands.simpleHoodZero(shooter),
                                        ShooterCommands.shooterIdle(shooter)
                                ).withName("Default Command"),
                                shooter::hasZeroed
                        )

                );
                // CLIMB DEFAULT COMMAND
                climb.setDefaultCommand(sequence(
                        ClimbCommands.zero(climb, 10.0).withTimeout(5),
                        ClimbCommands.runClimb(climb, operatorController::getLeftY, operatorController::getRightY)
                ));

                leds.setDefaultCommand(
                        either(LEDCommands.enabled(leds), LEDCommands.disabled(leds, this), RobotState::isEnabled)
                        .ignoringDisable(true));

                // ---- DRIVETRAIN COMMANDS ----
                driverController.x().whileTrue(runOnce(drive::stopWithX, drive));

                var command =
                        DriveCommands.aimAtSpeakerCommand(
                                drive,
                                () -> (-driverController.getLeftY() * (invertX.get()?-1:1)),
                                () -> (-driverController.getLeftX() * (invertY.get()?-1:1)),
                                () -> (-driverController.getRightX()) * (invertOmega.get()?-1:1));

                driverController.leftBumper().whileTrue(command.getCommand());

                // ---- INTAKE COMMANDS ----
                driverController
                        .leftTrigger()
                        .whileTrue(
                                parallel(
                                        smartIntakeCommand(intake, feeder),
                                        feedToBeamBreak(feeder)
                                ))
                        .onFalse(
                                either(
                                        none(),
                                        race(
                                                feedToBeamBreak(feeder).withTimeout(5),
                                                flushIntakeWithoutTheArmExtendedOutward(intake, feeder)
                                        ),
                                        feeder::getBeamBroken
                                )
                        );

                operatorController
                        .povLeft()
                        .whileTrue(
                                IntakeCommands.flushIntake(intake)
                                        .alongWith(FeederCommands.flushFeeder(feeder))
                        );
                operatorController
                        .povDown()
                        .and(operatorController.a().negate())
                        .whileTrue(
                                sequence(
                                        parallel(
                                                ShooterCommands.humanPlayerIntake(shooter),
                                                FeederCommands.humanPlayerIntake(feeder)
                                        ).until(feeder::getBeamBroken),
                                        parallel(
                                                ShooterCommands.humanPlayerIntake(shooter),
                                                FeederCommands.humanPlayerIntake(feeder)
                                        ).until(() -> !feeder.getBeamBroken())
                                ).andThen(FeederCommands.feedToBeamBreak(feeder))
                        )
                        .onFalse(
                                FeederCommands.humanPlayerIntake(feeder)
                                        .withTimeout(5.0)
                                        .until(() -> !feeder.getBeamBroken())
                        );
                operatorController
                        .povUp()
                        .and(operatorController.a())
                        .whileTrue(
                                LEDCommands.ledsUp(leds)
                        );
                operatorController
                        .povDown()
                        .and(operatorController.a())
                        .whileTrue(
                                LEDCommands.ledsDown(leds)
                        );

                // ---- SHOOTER COMMANDS ----
                operatorController
                        .y()
                        .whileTrue(autoAim(shooter, drive, feeder));
                operatorController
                        .x()
                        .whileTrue(
                                ShooterCommands.JustShoot(shooter)
                        );
                operatorController
                        .leftBumper()
                        .onTrue(
                                feedToBeamBreak(feeder)
                                        .withTimeout(5)
                        );
                operatorController.rightTrigger()
                        .whileTrue(passNote(shooter));
                driverController
                        .rightTrigger()
                        .whileTrue(
                                sequence(
                                        waitUntil(() -> (shooter.allAtSetpoint() && (shooter.getShooterVelocityRPM() > 1000) /*&& command.getReadySupplier().getAsBoolean()*/)),
                                        feedToShooter(feeder)
                                                .until(() -> !feeder.getBeamBroken()),
                                        feedToShooter(feeder)
                                                .withTimeout(.25)
                                )
                        );
                operatorController.rightBumper().whileTrue(run(()->{feeder.runVolts(8);}, feeder));
                operatorController
                        .start()
                        .onTrue(
                                ShooterCommands.simpleHoodZero(shooter)
                                        .withTimeout(4.0)
                        );
                driverController
                        .rightBumper()
                        .whileTrue(
                                sequence(
                                        FeederCommands.feedToShooter(feeder)
                                                .alongWith(ShooterCommands.ampSpin(shooter)).withTimeout(0.2),
                                        ShooterCommands.ampAng(shooter)
                                                .alongWith(ShooterCommands.ampGo(shooter, 600))
                                                .withTimeout(0.25)
                                                .andThen(ShooterCommands.setAmpAngle(shooter, -0.4))
                                )
                        );
                break;
            case DriveMotors:
                drive.setDefaultCommand(
                        DriveCommands.joystickDrive(
                                drive,
                                driverController::getLeftY,
                                driverController::getLeftX,
                                driverController::getRightX));
                var drivetrainDriveSysID =
                        new SysIdRoutine(
                                new Config(Voltage.per(Units.Second).of(2), Voltage.of(8.0), Seconds.of(12.0)),
                                new Mechanism(
                                        drive::runCharacterizationVolts,
                                        drive::populateDriveCharacterizationData,
                                        drive,
                                        "DrivetrainDriveMotors"));
                driverController
                        .x()
                        .whileTrue(drivetrainDriveSysID.dynamic(Direction.kForward))
                        .onFalse(runOnce(drive::stopWithX, drive));
                driverController
                        .y()
                        .whileTrue(drivetrainDriveSysID.dynamic(Direction.kReverse))
                        .onFalse(runOnce(drive::stopWithX, drive));
                driverController
                        .a()
                        .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kForward).withTimeout(20.0))
                        .onFalse(runOnce(drive::stopWithX, drive));
                driverController
                        .b()
                        .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kReverse).withTimeout(20.0))
                        .onFalse(runOnce(drive::stopWithX, drive));
                driverController
                        .rightTrigger()
                        .whileTrue(
                                new RunCommand(() -> shooter.setTargetShooterAngle(new Rotation2d(-0.61)))
                                        .andThen(
                                                (new RunCommand(
                                                        () -> shooter.shooterRunVelocity(5000), //THIS NUMBER NEEDS TO BE CALIBRATED

                                                        intake))));


                break;
            case TurnMotors:
                break;
            case Shooter:
                var shooterSysId =
                        new SysIdRoutine(
                                new Config(Voltage.per(Units.Second).of(.25), Voltage.of(9.0), Seconds.of(36)),
                                new Mechanism(
                                        shooter::shooterRunVolts,
                                        (log) -> {
                                            var motor = log.motor("Shooter");
                                            motor.voltage(shooter.getCharacterizationAppliedVolts());
                                            motor.angularPosition(shooter.getCharacterizationPosition());
                                            motor.angularVelocity(shooter.getCharacterizationVelocity());
                                            motor.current(shooter.getCharacterizationCurrent());
                                        },
                                        climb,
                                        "FlywheelMotors"));
                driverController
                        .a()
                        .onTrue(
                                shooterSysId
                                        .dynamic(Direction.kForward)
                                        .withTimeout(3)
                                        .andThen(
                                                new WaitCommand(5),
                                                shooterSysId.dynamic(Direction.kReverse).withTimeout(3),
                                                new WaitCommand(5),
                                                shooterSysId.quasistatic(Direction.kForward).withTimeout(36),
                                                new WaitCommand(5),
                                                shooterSysId.quasistatic(Direction.kReverse).withTimeout(36))
                                        .alongWith(new RunCommand(() -> {
                                            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0.0));
                                            shooter.setCharacterizeMode(true);
                                        })));
                break;
            case AllElse:
                var shooterArmSysID =
                        new SysIdRoutine(
                                new Config(Voltage.per(Units.Second).of(1), Voltage.of(9.0), Seconds.of(5)),
                                new Mechanism(
                                        shooter::runHoodVoltage,
                                        (log) -> {
                                            var motor = log.motor("FeederKraken");
                                            motor
                                                    .voltage(shooter.getHoodCharacterizationVoltage())
                                                    .angularPosition(shooter.getHoodCharacterizationPosition())
                                                    .angularVelocity(shooter.getHoodCharacterizationVelocity());
                                        },
                                        climb,
                                        "FeederMotors"));
                var intakeWheelsSysID =
                        new SysIdRoutine(
                                new Config(Voltage.per(Units.Second).of(1), Voltage.of(9.0), Seconds.of(9)),
                                new Mechanism(
                                        intake::runArmVolts,
                                        (log) -> {
                                            var motor = log.motor("IntakeWheels");
                                            motor.voltage(intake.getArmCharacterizationVoltage());
                                            motor.angularPosition(intake.getArmCharacterizationPosition());
                                            motor.angularVelocity(intake.getArmCharacterizationVelocity());
                                            motor.current(intake.getArmCharacterizationCurrent());
                                        },
                                        intake,
                                        "IntakeWheels"));
                driverController.a().onTrue(
                        sequence(
                                shooterArmSysID.quasistatic(Direction.kForward).until(() -> shooter.getHoodCharacterizationPosition().gte(Radians.of(1.8)) && shooter.getHoodCharacterizationPosition().lte(Radians.of(-1.6))), //fixme shooterArmSysID has incorrect name (intake arm sysid)
                                shooterArmSysID.quasistatic(Direction.kReverse).until(() -> shooter.getHoodCharacterizationPosition().gte(Radians.of(1.8)) && shooter.getHoodCharacterizationPosition().lte(Radians.of(-1.6))),
                                shooterArmSysID.dynamic(Direction.kForward).until(() -> shooter.getHoodCharacterizationPosition().gte(Radians.of(1.8)) && shooter.getHoodCharacterizationPosition().lte(Radians.of(-1.6))),
                                shooterArmSysID.dynamic(Direction.kReverse).until(() -> shooter.getHoodCharacterizationPosition().gte(Radians.of(1.8)) && shooter.getHoodCharacterizationPosition().lte(Radians.of(-1.6)))
                        )
                );
//                                intakeWheelsSysID.quasistatic(Direction.kForward),
//                                intakeWheelsSysID.quasistatic(Direction.kReverse),
//                                intakeWheelsSysID.dynamic(Direction.kForward),
//                                intakeWheelsSysID.dynamic(Direction.kReverse)
        }
    }

    // TODO: populate switch statements here
    public Command getEnterCommand(Mode m) {
        return none();
    }

    public Command getExitCommand(Mode m) {
        return none();
    }

    public void configureReactions() {
        driverRumble.setDefaultCommand(noRumble(driverRumble).ignoringDisable(true));
        operatorRumble.setDefaultCommand(noRumble(operatorRumble).ignoringDisable(true));
        reactions.intakeBeamBroken
                .and(reactions.shooterBeamBroken.negate())
                .whileTrue(
                        parallel(
                                parallel(
                                        rumbleLight(driverRumble)
                                                .withTimeout(0.1),
                                        waitSeconds(0.2)
                                                .andThen(rumbleLightWithFalloff(operatorRumble).withTimeout(10.0))),
                                LEDCommands.hasNote(leds)
                                        .withTimeout(1.0)
                                        .andThen(
                                                LEDCommands.setIntakeType(leds)
                                        )
                        ).ignoringDisable(true)
                );
        reactions
                .isAutonomous
                .and(reactions.isEnabled)
                .whileTrue(LEDCommands.flameCommand(leds).ignoringDisable(true));
        reactions
                .isTeleop
                .and(reactions.isEnabled)
                .whileTrue(LEDCommands.enabled(leds).ignoringDisable(true));
        reactions
                .isTeleop
                .and(reactions.isEnabled)
                .whileTrue(ClimbCommands.zeroClimb(climb).ignoringDisable(true).withTimeout(6.5));
    }


    enum SmartCommandsMode {
        Smart,
        Safe,
        Disabled
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public enum SysIDMode {
        Disabled,
        DriveMotors,
        TurnMotors,
        Shooter,
        AllElse
    }

    private static class ReactionObject {
        Trigger isEnabled;
        Trigger intakeBeamBroken;
        Trigger shooterBeamBroken;
        Trigger shooterAllAtSetpoint;
        Trigger shooterHoodAtSetpoint;
        Trigger shooterRpmAtSetpoint;
        Trigger isTeleop;
        Trigger isAutonomous;

        public ReactionObject(Trigger intakeBeamBroken, Trigger shooterBeamBroken, Trigger shooterAllAtSetpoint, Trigger shooterHoodAtSetpoint, Trigger shooterRpmAtSetpoint, Trigger t, Trigger b, Trigger trigger) {
            this.intakeBeamBroken = intakeBeamBroken;
            this.shooterBeamBroken = shooterBeamBroken;
            this.shooterAllAtSetpoint = shooterAllAtSetpoint;
            this.shooterHoodAtSetpoint = shooterHoodAtSetpoint;
            this.shooterRpmAtSetpoint = shooterRpmAtSetpoint;
            this.isTeleop = t;
            this.isAutonomous = b;
            this.isEnabled = trigger;
        }
    }
}

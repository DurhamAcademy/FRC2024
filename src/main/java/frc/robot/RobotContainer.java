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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.*;
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
import frc.robot.util.Dashboard;
import frc.robot.util.Mode;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.BaseUnits.Voltage;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.commands.FeederCommands.feedToShooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static SysIDMode sysIDMode = SysIDMode.Disabled;
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;
    private final Climb climb;
    private final VisionIOReal vision;
    private final Dashboard dashboard;
    private final LEDs leds;

//  private final ModeHelper modeHelper = new ModeHelper(this);
// Controller
private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardNumber flywheelSpeedInput =
            new LoggedDashboardNumber("Flywheel Speed", 1500.0);
    LoggedDashboardNumber angleOffsetInput = new LoggedDashboardNumber("Angle Offset", 0.0);
    // Subsystems
    private Drive drive;

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
                                new VisionIOReal("ShootSideCamera"),
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                shooter = new Shooter(new ShooterIOTalonFX(), new HoodIOSparkMax()); // new HoodIOSparkMax() {}
                feeder = new Feeder(new FeederIOTalonFX());
                intake = new Intake(new IntakeIOSparkMax());
                climb = new Climb(new ClimbIOSparkMax());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {
                                },
                                new VisionIOSim(
                                        "ShootSideCamera", () -> (drive == null) ? (drive.getPose()) : new Pose2d()),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
                shooter = new Shooter(new ShooterIOSim(), new HoodIO() {
                });
                feeder = new Feeder(new FeederIOSim());
                intake = new Intake(new IntakeIOSim());
                climb = new Climb(new ClimbIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {
                                },
                                new VisionIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                });
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
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> driverController.getRightX());
        NamedCommands.registerCommand(
                "AutoShoot", none()
//                ShooterCommands.autoAim(shooter, drive).alongWith(
//                        sequence(
//                                Commands.waitUntil(shooter::allAtSetpoint),
//                                FeederCommands.feedToShooter(feeder)
        );
        // Set up auto routines
        NamedCommands.registerCommand(
                "Shoot", none()
//                sequence(
//                        FeederCommands.feedToBeamBreak(feeder, intake),
//                        Commands.waitUntil(() -> (shooter.allAtSetpoint() && (shooter.getShooterVelocityRPM() > 1000))),
//                        FeederCommands.feedToShooter(feeder)
        );
//                        .deadlineWith(ShooterCommands.JustShoot(shooter))
//                        .withTimeout(8.0));
        NamedCommands.registerCommand(
                "Intake", none()
//                IntakeCommands.intakeCommand(intake).withTimeout(4.0)
//                        .until(feeder::getBeamBroken)
        );
        NamedCommands.registerCommand(
                "Drive Backwards", none()
//                DriveCommands.joystickDrive(drive, () -> .1, () -> 0, () -> 0.0).withTimeout(4.0)
        );
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        vision = new VisionIOReal("ShootSideCamera");
        dashboard = new Dashboard(autoChooser, drive, shooter, feeder, intake, vision);

        configureButtonBindings();
    }

    // TODO: populate switch statements here
    public Command getEnterCommand(Mode m) {
        return none();
    }

    public Command getExitCommand(Mode m) {
        return none();
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
                                () -> driverController.getLeftY(),
                                () -> driverController.getLeftX(),
                                () -> driverController.getRightX()));
                intake.setDefaultCommand(IntakeCommands.idleCommand(intake));
                feeder.setDefaultCommand(new RunCommand(() -> feeder.runVolts(0.0), feeder));
                shooter.setDefaultCommand(
                        either(
                                ShooterCommands.shooterIdle(shooter),
                                sequence(
                                        ShooterCommands.shooterIdle(shooter).until(shooter::hoodAtSetpoint),
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

                // ---- DRIVETRAIN COMMANDS ----
                driverController.x().whileTrue(runOnce(drive::stopWithX, drive));


                var command =
                        DriveCommands.aimAtSpeakerCommand(
                                drive,
                                driverController::getLeftY,
                                driverController::getLeftX,
                                driverController::getRightX);
                driverController.leftBumper().whileTrue(command.getCommand());

                // ---- INTAKE COMMANDS ----
                driverController
                        .leftTrigger()
                        .whileTrue(
                                parallel(
                                        sequence(
                                                IntakeCommands.intakeCommand(intake).until(feeder::getIntakeBeamBroken),
                                                IntakeCommands.flushIntake(intake)
                                        ),
                                        FeederCommands.feedToBeamBreak(feeder)
                                ))
                        .onFalse(
                                race(
                                        FeederCommands.feedToBeamBreak(feeder).withTimeout(5),
                                        IntakeCommands.flushIntakeWithoutTheArmExtendedOutward(intake)
                                )
                        );
                driverController.rightBumper().whileTrue(IntakeCommands.idleCommand(intake));

                operatorController
                        .povDown()
                        .whileTrue(
                                IntakeCommands.flushIntake(intake)
                                        .alongWith(FeederCommands.flushFeeder(feeder))
                        );

                // ---- SHOOTER COMMANDS ----
                operatorController
                        .y()
                        .whileTrue(
//                        either(
//                                sequence(
//                                        ShooterCommands.autoAim(shooter, drive)
//                                                .until(() -> !feeder.getBeamBroken()),
//                                        ShooterCommands.autoAim(shooter, drive)
//                                                .withTimeout(0.25)
//                                ),
                                ShooterCommands.autoAim(shooter, drive)//,
//                                () -> !feeder.getBeamBroken()
//                        )
                        );
                operatorController
                        .x()
                        .whileTrue(
                                ShooterCommands.JustShoot(shooter)
                        );
                operatorController
                        .a()
                        .whileTrue(
                                parallel(
                                        sequence(
                                                ShooterCommands.ampShoot(shooter).until(() -> !feeder.getBeamBroken()),
                                                ShooterCommands.pushIntoAmp(shooter)
                                        ),
                                        sequence(
                                                waitSeconds(0.5),
                                                waitUntil(shooter::allAtSetpoint),
                                                FeederCommands.feedToShooter(feeder)
                                                        .until(() -> !feeder.getBeamBroken())
                                        )

                                )
                        );
                operatorController
                        .leftBumper()
                        .whileTrue(
                                FeederCommands.feedToBeamBreak(feeder)
                        );
                driverController
                        .rightTrigger()
                        .whileTrue(
                                sequence(
                                        waitUntil(() -> (shooter.allAtSetpoint() && (shooter.getShooterVelocityRPM() > 1000) && command.getReadySupplier().getAsBoolean())),
                                        feedToShooter(feeder)
                                                .until(() -> !feeder.getBeamBroken()),
                                        feedToShooter(feeder)
                                                .withTimeout(.25)
                                )
                        );
                operatorController
                        .start()
                        .onTrue(
                                ShooterCommands.addToOffset()
                        );
                operatorController
                        .back()
                        .onTrue(
                                ShooterCommands.removeFromOffset()
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
                                new Config(Voltage.per(Units.Second).of(.5), Voltage.of(8.0), Seconds.of(12.0)),
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
                        .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kForward).withTimeout(2.0))
                        .onFalse(runOnce(drive::stopWithX, drive));
                driverController
                        .b()
                        .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kReverse).withTimeout(2.0))
                        .onFalse(runOnce(drive::stopWithX, drive));
                driverController
                        .rightTrigger()
                        .whileTrue(
                                new RunCommand(() -> shooter.setTargetShooterAngle(new Rotation2d(-0.61)))
                                        .andThen(
                                                (new RunCommand(
                                                        () -> shooter.shooterRunVelocity(5000), //THIS NUMBER NEEDS TO BE CALIBRATED

                                                        intake))));
                operatorController
                        .povUp()
                        .whileTrue(
                                sequence(
                                    parallel(
                                            ShooterCommands.humanPlayerIntake(shooter),
                                            FeederCommands.humanPlayerIntake(feeder)
                                    )
                                            .until(() -> feeder.getBeamBroken()),
                                        parallel(
                                                ShooterCommands.humanPlayerIntake(shooter),
                                                FeederCommands.humanPlayerIntake(feeder)
                                        )
                                                .until(() -> !feeder.getBeamBroken())
                                )
                        );
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
}

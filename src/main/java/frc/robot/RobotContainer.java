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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.BaseUnits.Voltage;
import static edu.wpi.first.units.Units.Seconds;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Intake intake;
  private Climb climb;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        shooter = new Shooter(new ShooterIOTalonFX());
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIOSparkMax());
        climb = new Climb(new ClimbIOSparkMax());
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        shooter = new Shooter(new ShooterIOSim());
        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim());
        climb = new Climb(new ClimbIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> shooter.runVelocity(flywheelSpeedInput.get()), shooter::stop, shooter)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  public static SysIDMode sysIDMode = SysIDMode.Disabled;

  public enum SysIDMode {
    Disabled,
    DriveMotors,
    TurnMotors,
    EverythingElse
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    switch (sysIDMode) {
      case Disabled:
        // ---- DEFAULT COMMANDS ----
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));
        intake.setDefaultCommand(
            new RunCommand(
                () -> {
                  intake.setIntakePosition(Rotation2d.fromDegrees(-130));
                  intake.setRollerPercentage(0.0);
                },
                intake));
        feeder.setDefaultCommand(new RunCommand(() -> feeder.runVolts(6.0), feeder));
        shooter.setDefaultCommand(new RunCommand(shooter::stop, shooter));
        // CLIMB DEFAULT COMMAND
        climb.setDefaultCommand(
            Commands.run(
                () -> {
                  climb.runLeftVolts(
                      MathUtil.applyDeadband(operatorController.getLeftY(), 0.075) * 12);
                  climb.runRightVolts(operatorController.getRightY() * 12);
                },
                climb));

        // ---- DRIVETRAIN COMMANDS ----
        driverController.x().whileTrue(Commands.runOnce(drive::stopWithX, drive));
        driverController
            .b()
            .whileTrue(
                Commands.runOnce(
                        () -> {
                          try {
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                          } catch (Drive.GyroConnectionException ignored) {
                          }
                        },
                        drive)
                    .ignoringDisable(true));

        // ---- FEEDER COMMANDS ----
        driverController
            .leftTrigger()
            .and(feeder::getSensorFeed)
            .whileTrue(
                new RunCommand(() -> feeder.runVolts(6.0), feeder)
                    .until(() -> !feeder.getSensorFeed()));

        // ---- INTAKE COMMANDS ----
        driverController
                .leftTrigger() // not a()
                .whileTrue(IntakeCommands.intakeCommand(intake).alongWith(IntakeCommands.feedToBeamBreak(feeder)))
                .onFalse(IntakeCommands.feedToBeamBreak(feeder).withTimeout(5));
        driverController.rightBumper().whileTrue(IntakeCommands.idleCommand(intake));

        // ---- SHOOTER COMMANDS ----
        driverController
            .rightTrigger()
            .whileTrue(
                Commands.run(
                    () -> shooter.runVolts(driverController.getRightTriggerAxis() * 12.0),
                    shooter));
        driverController.leftTrigger().and(driverController.rightTrigger().negate());
        driverController
            .a()
            .whileTrue(
                Commands.startEnd(
                    () -> shooter.runVelocity(flywheelSpeedInput.get()), shooter::stop, shooter));
        driverController
            .rightTrigger()
            .onTrue(new RunCommand(() -> shooter.runVolts(6.0), shooter));
        driverController
            .a()
            .whileTrue(
                new RunCommand(
                    () -> shooter.runVolts(driverController.getLeftTriggerAxis() * 12.0), shooter));

        break;
      case DriveMotors:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));
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
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        driverController
            .y()
            .whileTrue(drivetrainDriveSysID.dynamic(Direction.kReverse))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        driverController
            .a()
            .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kForward).withTimeout(2.0))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        driverController
            .b()
            .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kReverse).withTimeout(2.0))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        driverController
            .rightTrigger()
            .whileTrue(
                    new RunCommand(() -> shooter.setTargetShooterAngle(new Rotation2d(-0.61)))
                    .andThen(
                        (new RunCommand(
                            () -> shooter.runVelocity(5000) /*THIS NUMBER NEEDS TO BE CALIBRATED*/,
                            intake))));
        break;
      case EverythingElse:
        var shooterSysId =
                new SysIdRoutine(
                        new Config(Voltage.per(Units.Second).of(.1), Voltage.of(9.0), Seconds.of(120)),
                        new Mechanism(
                                shooter::runVoltage,
                                (log) -> {
                                  var motor = log.motor("Shooter");
                                  motor.voltage(shooter.getCharacterizationVoltage());
                                  motor.angularPosition(shooter.getCharacterizationPosition());
                                  motor.angularVelocity(shooter.getCharacterizationVelocity());
                                  motor.current(shooter.getCharacterizationCurrent());
                                },
                                shooter,
                                "FlywheelMotors"));
        driverController
                .a()
                .onTrue(
                        shooterSysId
                                .dynamic(Direction.kForward)
                                .withTimeout(5)
                                .andThen(
                                        new WaitCommand(5),
                                        shooterSysId.dynamic(Direction.kReverse).withTimeout(5),
                                        new WaitCommand(5),
                                        shooterSysId.quasistatic(Direction.kForward).withTimeout(120),
                                        new WaitCommand(5),
                                        shooterSysId.quasistatic(Direction.kReverse).withTimeout(120)));
        break;
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
}

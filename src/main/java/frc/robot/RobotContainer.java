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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Mode;
import frc.robot.util.ModeHelper;
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

  private final ModeHelper modeHelper = new ModeHelper(this);

  // TODO: populate switch statements here
  public Command getEnterCommand(Mode m) {
    return new InstantCommand();
  }

  public Command getExitCommand(Mode m) {
    return new InstantCommand();
  }

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

    public static SysIDMode sysIDMode = SysIDMode.Disabled;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSim(/*0*/ ),
                new ModuleIOSim(/*1*/ ),
                new ModuleIOSim(/*2*/ ),
                new ModuleIOSim(/*3*/ ));
        shooter = new Shooter(new ShooterIOTalonFX(), new HoodIOSparkMax() {});
        feeder = new Feeder(new FeederIO() {});
          intake = new Intake(new IntakeIOSparkMax() {
          });
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
        shooter = new Shooter(new ShooterIOSim(), new HoodIO() {});
        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim());
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
        shooter = new Shooter(new ShooterIO() {}, new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> shooter.shooterRunVelocity(flywheelSpeedInput.get()),
                shooter::stopShooter,
                shooter)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

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
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
        intake.setDefaultCommand(
            new RunCommand(
                () -> {
                  intake.setIntakePosition(Rotation2d.fromDegrees(-130));
                  intake.setRollerPercentage(0.0);
                },
                intake));
        feeder.setDefaultCommand(new RunCommand(feeder::stop, feeder));

        // ---- DRIVETRAIN COMMANDS ----
        controller.x().whileTrue(Commands.runOnce(drive::stopWithX, drive));
        controller
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
        controller
            .leftTrigger()
            .and(feeder::getSensorFeed)
            .whileTrue(
                new RunCommand(() -> feeder.runVolts(6.0), feeder)
                    .until(() -> !feeder.getSensorFeed()));

        // prepare the shooter for dumping into the amp
        controller.a().onTrue(Commands.runOnce(() -> modeHelper.switchTo(Mode.AMP)));

        /*
            .toggleOnTrue(
                Commands.sequence(
                    Commands.run(
                            () -> shooter.setTargetShooterAngleRad(Rotation2d.fromDegrees(-22.5)),
                            shooter)
                        .until(() -> false),
                    Commands.run(() -> feeder.runVolts(6.0), feeder)
                        .withTimeout(2.0)
                        .until(() -> !feeder.getSensorFeed()),
                    Commands.runOnce(feeder::stop, feeder),
                    Commands.run(
                        () -> shooter.setTargetShooterAngleRad(Rotation2d.fromDegrees(45.0)))));
        */
        // ---- INTAKE COMMANDS ----
        controller
            .leftBumper() // not a()
            .whileTrue(
                new RunCommand(
                    () -> {
                      intake.setIntakePosition(Rotation2d.fromDegrees(0.0));
                      intake.setRollerPercentage(0.75);
                    },
                    intake));
        controller
            .rightBumper()
            .whileTrue(
                new RunCommand(
                    () -> {
                      intake.setIntakePosition(Rotation2d.fromDegrees(-90.0));
                      intake.setRollerPercentage(0.0);
                    },
                    intake));

        // ---- SHOOTER COMMANDS ----
        controller
            .a()
            .whileTrue(
                Commands.startEnd(
                    () -> shooter.shooterRunVelocity(flywheelSpeedInput.get()),
                    shooter::stopShooter,
                    shooter));
        controller
            .rightTrigger()
            .whileTrue(
                new StartEndCommand(
                    () -> shooter.shooterRunVolts(12.0 * controller.getRightTriggerAxis()),
                    () -> {
                      shooter.stopShooter();
                      shooter.shooterRunVolts(0.0);
                    },
                    shooter));
        controller
            .a()
            .whileTrue(
                Commands.startEnd(
                    () -> shooter.shooterRunVelocity(flywheelSpeedInput.get()),
                    shooter::stopShooter,
                    shooter));

        break;
      case DriveMotors:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
        var drivetrainDriveSysID =
            new SysIdRoutine(
                new Config(Voltage.per(Units.Second).of(.5), Voltage.of(8.0), Seconds.of(12.0)),
                new Mechanism(
                    drive::runCharacterizationVolts,
                    drive::populateDriveCharacterizationData,
                    drive,
                    "DrivetrainDriveMotors"));
        controller
            .x()
            .whileTrue(drivetrainDriveSysID.dynamic(Direction.kForward))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        controller
            .y()
            .whileTrue(drivetrainDriveSysID.dynamic(Direction.kReverse))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        controller
            .a()
            .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kForward).withTimeout(2.0))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        controller
            .b()
            .whileTrue(drivetrainDriveSysID.quasistatic(Direction.kReverse).withTimeout(2.0))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        controller
            .rightTrigger()
            .whileTrue(
                new RunCommand(() -> shooter.setTargetShooterAngleRad(new Rotation2d(-0.61)))
                    .andThen(
                        (new RunCommand(
                            () ->
                                shooter.shooterRunVelocity(
                                    5000) /*THIS NUMBER NEEDS TO BE CALIBRATED*/,
                            intake))));
        break;
      case EverythingElse:
        var shooterSysId =
            new SysIdRoutine(
                    new Config(Voltage.per(Units.Second).of(.1), Voltage.of(9.0), Seconds.of(120)),
                new Mechanism(
                    shooter::shooterRunVolts,
                    (log) -> {
                      var motor = log.motor("Shooter");
                      motor.voltage(shooter.getCharacterizationAppliedVolts());
                        motor.angularPosition(shooter.getCharacterizationPosition());
                      motor.angularVelocity(shooter.getCharacterizationVelocity());
                      motor.current(shooter.getCharacterizationCurrent());
                    },
                    shooter,
                    "FlywheelMotors"));
        controller
                .a()
                .onTrue(
                        shooterSysId.dynamic(Direction.kForward).withTimeout(5)
                                .andThen(
                                        new WaitCommand(5),
                                        shooterSysId.dynamic(Direction.kReverse).withTimeout(5),
                                        new WaitCommand(5),
                                        shooterSysId.quasistatic(Direction.kForward).withTimeout(120),
                                        new WaitCommand(5),
                                        shooterSysId.quasistatic(Direction.kReverse).withTimeout(120)
                                )
                );
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

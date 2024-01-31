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

import static edu.wpi.first.units.BaseUnits.Voltage;
import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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
  private Intake intake; // final?

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Shooter Speed", 1500.0);

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
        shooter = new Shooter(new ShooterIOSparkMax());
        feeder = new Feeder(new FeederIO() {});
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // shooter = new Shooter(new ShooterIOTalonFX());
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
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Shooter",
        Commands.startEnd(
                () -> shooter.runVelocity(flywheelSpeedInput.get()), shooter::stop, shooter)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    //    autoChooser.addOption(
    //        "Drive FF Characterization",
    //        new FeedForwardCharacterization(
    //            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    //    autoChooser.addOption(
    //        "Flywheel FF Characterization",
    //        new FeedForwardCharacterization(
    //            flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  public SysIDMode sysIDMode = SysIDMode.Disabled;

  enum SysIDMode {
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
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
        controller
            .leftTrigger()
            .and(feeder::getSensorFeed)
            .onTrue(
                new RunCommand(() -> feeder.runVolts(6.0)).until(() -> !feeder.getSensorFeed()));
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive)
                    .ignoringDisable(true));

        controller // intake motor
            .leftTrigger() // not a()
            .onTrue(new RunCommand(() -> intake.setIntakePosition(new Rotation2d(115.0))));
        controller.leftTrigger().onTrue(new RunCommand(() -> intake.setRollerPercentage(0.75)));
        controller
            .a()
            .whileTrue(
                Commands.startEnd(
                    () -> shooter.runVelocity(flywheelSpeedInput.get()), shooter::stop, shooter));
        controller.rightTrigger().onTrue(new RunCommand(() -> shooter.runVolts(6.0)));
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
        break;
      case TurnMotors:
        var drivetrainTurnSysID =
            new SysIdRoutine(
                new Config(Voltage.per(Units.Second).of(.5), Voltage.of(8.0), Seconds.of(12.0)),
                new Mechanism(
                    drive::runCharacterizationVolts,
                    drive::populateTurnCharacterizationData,
                    drive,
                    "DrivetrainDriveMotors"));
        controller
            .x()
            .whileTrue(
                drivetrainTurnSysID
                    .dynamic(Direction.kForward)
                    .andThen(drivetrainTurnSysID.dynamic(SysIdRoutine.Direction.kReverse))
                    .andThen(drivetrainTurnSysID.quasistatic(Direction.kForward))
                    .andThen(drivetrainTurnSysID.quasistatic(Direction.kReverse))
                    .andThen(Commands.runOnce(drive::stopWithX, drive)))
            .onFalse(Commands.runOnce(drive::stopWithX, drive));
        break;
      case EverythingElse:
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

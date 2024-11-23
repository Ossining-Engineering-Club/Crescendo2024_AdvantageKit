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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.amp.AngleAmpPivot;
import frc.robot.commands.intake.IntakeNoteToShooter;
import frc.robot.commands.intake.ReverseNote;
import frc.robot.commands.pivot.AnglePivot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.amppivot.AmpPivot;
import frc.robot.subsystems.amppivot.AmpPivotConstants;
import frc.robot.subsystems.amppivot.AmpPivotIO;
import frc.robot.subsystems.amppivot.AmpPivotIOSim;
import frc.robot.subsystems.amppivot.AmpPivotIOSparkMax;
import frc.robot.subsystems.breakbeam.BreakbeamIO;
import frc.robot.subsystems.breakbeam.BreakbeamIOReal;
import frc.robot.subsystems.breakbeam.BreakbeamIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXAndSparkMax;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSparkMax;
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
  private final Flywheel flywheel;
  private final AmpPivot ampPivot;
  private final Feeder feeder;
  private final Intake intake;
  private final Pivot pivot;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);

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
                new ModuleIOTalonFXAndSparkMax(0),
                new ModuleIOTalonFXAndSparkMax(1),
                new ModuleIOTalonFXAndSparkMax(2),
                new ModuleIOTalonFXAndSparkMax(3));
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        ampPivot = new AmpPivot(new AmpPivotIOSparkMax());
        feeder = new Feeder(new FeederIOSparkMax(), new BreakbeamIOReal(0));
        intake = new Intake(new IntakeIOSparkMax(), new BreakbeamIOReal(1));
        pivot = new Pivot(new PivotIOSparkMax());
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
        flywheel = new Flywheel(new FlywheelIOSim());
        ampPivot = new AmpPivot(new AmpPivotIOSim());
        feeder = new Feeder(new FeederIOSim(), new BreakbeamIOSim());
        intake = new Intake(new IntakeIOSim(), new BreakbeamIOSim());
        pivot = new Pivot(new PivotIOSim());
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
        flywheel = new Flywheel(new FlywheelIO() {});
        ampPivot = new AmpPivot(new AmpPivotIO() {});
        feeder = new Feeder(new FeederIO() {}, new BreakbeamIO() {});
        intake = new Intake(new IntakeIO() {}, new BreakbeamIO() {});
        pivot = new Pivot(new PivotIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // cancel everything
    secondaryController
        .povDown()
        .onTrue(
            Commands.runOnce(() -> stopEverything(), intake, feeder, flywheel, pivot, ampPivot));

    secondaryController.a().onTrue(new IntakeNoteToShooter(intake, feeder));

    secondaryController.b().onTrue(new AnglePivot(PivotConstants.kSubwooferAngle, pivot));
    secondaryController
        .b()
        .onTrue(
            Commands.runOnce(() -> flywheel.runVelocity(FlywheelConstants.kDefaultRPM), flywheel));

    secondaryController
        .y()
        .onTrue(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new AnglePivot(PivotConstants.kAmpAngle, pivot),
                    new AngleAmpPivot(AmpPivotConstants.extendedAngle, ampPivot)),
                new SequentialCommandGroup(
                    new AngleAmpPivot(AmpPivotConstants.stowAngle, ampPivot),
                    new AnglePivot(PivotConstants.kStartAngle, pivot)),
                () -> pivot.atAmpAngle()));

    secondaryController
        .rightBumper()
        .onTrue(
            new Shoot(FlywheelConstants.kDefaultRPM, flywheel, feeder)
                .andThen(new AnglePivot(PivotConstants.kStartAngle, pivot)));

    secondaryController.rightTrigger(0.9).whileTrue(new ReverseNote(intake, feeder));
  }

  public void stopEverything() {
    pivot.stop();
    intake.stop();
    flywheel.stop();
    feeder.stop();
    ampPivot.stop();
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

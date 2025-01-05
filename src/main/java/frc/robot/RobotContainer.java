// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.Feeder;

public class RobotContainer {

  public final Intake INTAKE_SUBSYSTEM;
  public final Shooter SHOOTER_SUBSYSTEM;
  public final Pivot PIVOT_SUBSYSTEM;
  public final Feeder FEEDER_SUBSYSTEM;

  public static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
      DriveSubsystem.initializeHardware(),
      Constants.Drive.DRIVE_ROTATE_PID,
      Constants.Drive.DRIVE_CONTROL_CENTRICITY,
      Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_SCALAR,
      Constants.HID.CONTROLLER_DEADBAND,
      Constants.Drive.DRIVE_LOOKAHEAD);

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(
      Constants.HID.PRIMARY_CONTROLLER_PORT);

  private final SendableChooser<Command> automodeChooser;

  public RobotContainer() {
    // Initialize subsystems
    INTAKE_SUBSYSTEM = new Intake();
    SHOOTER_SUBSYSTEM = new Shooter();
    PIVOT_SUBSYSTEM = new Pivot();
    FEEDER_SUBSYSTEM = new Feeder();

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
        DRIVE_SUBSYSTEM.driveCommand(
            PRIMARY_CONTROLLER::getLeftY,
            PRIMARY_CONTROLLER::getLeftX,
            PRIMARY_CONTROLLER::getRightX));

    // Register named commands
    registerNamedCommands();

    // Set up AutoBuilder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    // Bind buttons and triggers
    configureBindings();

    // Set up the auto chooser
    automodeChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

    // Initialize autos
    initializeAutos();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", FEEDER_SUBSYSTEM.feedNote().alongWith(INTAKE_SUBSYSTEM.runIntake()));
    NamedCommands.registerCommand("Pivot to Speaker", PIVOT_SUBSYSTEM.pivotPresetSpeaker());
    NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> SHOOTER_SUBSYSTEM.toggleState(ShooterState.SPEAKER), SHOOTER_SUBSYSTEM)
    .andThen(FEEDER_SUBSYSTEM.shootNote())
    .andThen(Commands.runOnce(() -> SHOOTER_SUBSYSTEM.toggleState(ShooterState.IDLE))));
  }

  private void initializeAutos() {
    PathPlannerAuto leaveAuto = new PathPlannerAuto("Leave");
    PathPlannerAuto preLoad1 = new PathPlannerAuto("Preload + 1");
    PathPlannerAuto preLoad3 = new PathPlannerAuto("Preload + 1");
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.toggleTractionControlCommand());


    // Intake + Outtake
    PRIMARY_CONTROLLER.leftBumper().whileTrue(
      FEEDER_SUBSYSTEM.feedNote()
      .alongWith(INTAKE_SUBSYSTEM.runIntake()))
      .whileFalse(INTAKE_SUBSYSTEM.runStop());

    PRIMARY_CONTROLLER.rightBumper().whileTrue(
      FEEDER_SUBSYSTEM.spitNote()
      .alongWith(INTAKE_SUBSYSTEM.runOuttake()))
      .whileFalse(INTAKE_SUBSYSTEM.runStop());

    // pivot up/down
    PRIMARY_CONTROLLER.leftTrigger().whileTrue(
        PIVOT_SUBSYSTEM.pivotUp()).whileFalse(PIVOT_SUBSYSTEM.pivotIdle());

    PRIMARY_CONTROLLER.rightTrigger().whileTrue(
        PIVOT_SUBSYSTEM.pivotDown()).whileFalse(PIVOT_SUBSYSTEM.pivotIdle());

    // Toggle shooter state
    PRIMARY_CONTROLLER.y().onTrue(
        Commands.runOnce(() -> {
          SHOOTER_SUBSYSTEM.toggleState(ShooterState.SPEAKER);
        }, SHOOTER_SUBSYSTEM));   

    // Shoot note
    PRIMARY_CONTROLLER.a().whileTrue(FEEDER_SUBSYSTEM.shootNote());

    // B button - go to source
    //PRIMARY_CONTROLLER.b().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.SOURCE));

    // Reset pose
    PRIMARY_CONTROLLER.povLeft().onTrue(DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new));

    // Pivot presets
    PRIMARY_CONTROLLER.povUp().whileTrue(PIVOT_SUBSYSTEM.pivotPresetAMP());
    PRIMARY_CONTROLLER.povDown().whileTrue(PIVOT_SUBSYSTEM.pivotPresetSpeaker());

    // Reset heading
    PRIMARY_CONTROLLER.rightStick().onTrue(Commands.runOnce(DRIVE_SUBSYSTEM.navx::reset, DRIVE_SUBSYSTEM));
  }

  /**
   * Run simulation related methods
   */
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /**
   * Get currently selected autonomous command
   * 
   * @return Autonomous command
   */
  public Command getAutonomousCommand() {
    return automodeChooser.getSelected();
  }
}

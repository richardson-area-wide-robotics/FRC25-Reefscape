// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {


  public static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
      DriveSubsystem.initializeHardware(),
      Constants.Drive.DRIVE_ROTATE_PID,
      Constants.Drive.DRIVE_CONTROL_CENTRICITY,
      Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_INPUT_CURVE,
      Angle.ofRelativeUnits(Constants.Drive.DRIVE_TURN_SCALAR, Units.Degree),
      Dimensionless.ofRelativeUnits(Constants.HID.CONTROLLER_DEADBAND, Units.Value),
      Time.ofRelativeUnits(Constants.Drive.DRIVE_LOOKAHEAD, Units.Second));

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(
      Constants.HID.PRIMARY_CONTROLLER_PORT);

  private final SendableChooser<Command> automodeChooser;

  public RobotContainer() {

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
        DRIVE_SUBSYSTEM.driveCommand(
            PRIMARY_CONTROLLER::getLeftY,
            PRIMARY_CONTROLLER::getLeftX,
            PRIMARY_CONTROLLER::getRightX));

    // Register named commands
    registerNamedCommands();


    // Bind buttons and triggers
    configureBindings();

    // Set up the auto chooser
    automodeChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

    // Initialize autos
    initializeAutos();
  }

  private void registerNamedCommands() {
    //NamedCommands.registerCommand("Intake", FEEDER_SUBSYSTEM.feedNote().alongWith(INTAKE_SUBSYSTEM.runIntake()));
  }

  private void initializeAutos() {
    //PathPlannerAuto leaveAuto = new PathPlannerAuto("Leave");
    //PathPlannerAuto preLoad1 = new PathPlannerAuto("Preload + 1");
    //PathPlannerAuto preLoad3 = new PathPlannerAuto("Preload + 1");
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.toggleTractionControlCommand());

    // Reset pose
    PRIMARY_CONTROLLER.povLeft().onTrue(DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new));

    // Reset heading
    PRIMARY_CONTROLLER.rightStick().onTrue(Commands.runOnce(DRIVE_SUBSYSTEM.navx::reset, DRIVE_SUBSYSTEM));
  }

  /**
   * Run simulation related methods
   */
  public void simulationPeriodic() {
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

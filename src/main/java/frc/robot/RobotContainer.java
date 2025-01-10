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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.interfaces.IRobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class RobotContainer implements IRobotContainer {


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
  private static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(
    Constants.HID.SECONDARY_CONTROLLER_PORT);

  private static SendableChooser<Command> automodeChooser = null; 

  @Override
  public RobotContainer createContainer(){
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

      return new RobotContainer();
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
    // Start - toggle traction control
    bindControl(PRIMARY_CONTROLLER.start(), DRIVE_SUBSYSTEM.toggleTractionControlCommand());

    // Left POV - Reset pose
    bindControl(PRIMARY_CONTROLLER.povLeft(), DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new));

    // Right Stick Button - Reset heading
    bindControl(PRIMARY_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM.navx::reset, DRIVE_SUBSYSTEM));
  }

  /**
   * Helper method to bind a control action to a command.
   *
   * @param control The button to bind to.
   * @param command The command to execute when that button is pressed.
   *
   * @author Hudson Strub
   * @since 2025
   */
  private static void bindControl(Trigger control, Command command) {
    control.onTrue(command);
  }

  /**
   * Run simulation related methods
   */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get currently selected autonomous command
   * 
   * @return Autonomous command
   */
  @Override
  public Command getAutonomousCommand() {
    return automodeChooser.getSelected();
  }
}

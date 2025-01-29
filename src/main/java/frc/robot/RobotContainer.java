// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HIDConstants;
import frc.robot.common.RobotUtils;
import frc.robot.common.annotations.Robot;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.DriveSubsystem;
import frc.robot.common.subsystems.shooter.KitBotShooter;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;


@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 1745) //Note: This class is also the defualt so it will be loaded on 8874
public class RobotContainer implements IRobotContainer {

  public static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
      DriveSubsystem.initializeHardware(),
      Constants.DriveConstants.DRIVE_ROTATE_PID,
      Constants.DriveConstants.DRIVE_CONTROL_CENTRICITY,
      Constants.DriveConstants.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.DriveConstants.DRIVE_TURN_INPUT_CURVE,
      Angle.ofRelativeUnits(Constants.DriveConstants.DRIVE_TURN_SCALAR, Units.Degree),
      Dimensionless.ofRelativeUnits(Constants.HIDConstants.CONTROLLER_DEADBAND, Units.Value),
      Time.ofRelativeUnits(Constants.DriveConstants.DRIVE_LOOKAHEAD, Units.Second));

  public static final KitBotShooter KIT_BOT_SHOOTER = new KitBotShooter(9);

  private static SendableChooser<Command> automodeChooser = null; 

  public static RobotContainer createContainer(){
        // Set drive command
        DRIVE_SUBSYSTEM.setDefaultCommand(
          DRIVE_SUBSYSTEM.driveCommand(
            HIDConstants.PRIMARY_CONTROLLER::getLeftY,
            HIDConstants.PRIMARY_CONTROLLER::getLeftX,
            HIDConstants.PRIMARY_CONTROLLER::getRightX));
  
      // Register named commands
      registerNamedCommands();
  
  
      // Bind buttons and triggers
      configureBindings();
  
      // Set up the auto chooser
      //automodeChooser = AutoBuilder.buildAutoChooser();
      // SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, automodeChooser);
  
      // Initialize autos
      initializeAutos();

      return new RobotContainer();
  }

  private static void registerNamedCommands() {
    //NamedCommands.registerCommand("Intake", FEEDER_SUBSYSTEM.feedNote().alongWith(INTAKE_SUBSYSTEM.runIntake()));
  }

  private static void initializeAutos() {
    //PathPlannerAuto leaveAuto = new PathPlannerAuto("Leave");
    //PathPlannerAuto preLoad1 = new PathPlannerAuto("Preload + 1");
    //PathPlannerAuto preLoad3 = new PathPlannerAuto("Preload + 1");
  }

  private static void configureBindings() {
    // Start - toggle traction control
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.start(), DRIVE_SUBSYSTEM.toggleTractionControlCommand(), Commands.none());

    // Left POV - Reset pose
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.povLeft(), DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new), Commands.none());

    // Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM.navx::reset, DRIVE_SUBSYSTEM), Commands.none());

    // Right Trigger - Outtake
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.rightTrigger(), KIT_BOT_SHOOTER.setSpeedCommand(1), KIT_BOT_SHOOTER.stopMotorCommand());

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

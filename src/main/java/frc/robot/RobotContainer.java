// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HIDConstants;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.annotations.Robot;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.DeepClimbSubsystem;
import frc.robot.common.subsystems.ElevatorSubsystem;
import frc.robot.common.subsystems.ScoringSubsystem;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;


@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 1745) //Note: This class is also the defualt so it will be loaded on 8874
public class RobotContainer implements IRobotContainer {

  public static final SwerveDriveSubsystem DRIVE_SUBSYSTEM = new SwerveDriveSubsystem(
      SwerveDriveSubsystem.initializeHardware(),
      Constants.DriveConstants.DRIVE_ROTATE_PID,
      Constants.DriveConstants.DRIVE_CONTROL_CENTRICITY,
      Constants.DriveConstants.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.DriveConstants.DRIVE_TURN_INPUT_CURVE,
      Angle.ofRelativeUnits(Constants.DriveConstants.DRIVE_TURN_SCALAR, Units.Degree),
      Dimensionless.ofRelativeUnits(Constants.HIDConstants.CONTROLLER_DEADBAND, Units.Value),
      Time.ofRelativeUnits(Constants.DriveConstants.DRIVE_LOOKAHEAD, Units.Second));

  public static final ElevatorSubsystem ELEVATOR_SUBSYSTEM = new ElevatorSubsystem(9);
  public static final DeepClimbSubsystem DEEP_CLIMB_SUBSYSTEM = new DeepClimbSubsystem(13, 14);
  public static final ScoringSubsystem SCORING_SUBSYSTEM = new ScoringSubsystem(15, 16);




  private static SendableChooser<Command> automodeChooser; 

  public static RobotContainer createContainer(){
        // Set drive command
        // LeftY is the xRequest and LeftX is the yRequest for some reason
        DRIVE_SUBSYSTEM.setDefaultCommand(
          DRIVE_SUBSYSTEM.driveCommand(
            HIDConstants.PRIMARY_CONTROLLER::getLeftY,
            HIDConstants.PRIMARY_CONTROLLER::getLeftX,
            HIDConstants.PRIMARY_CONTROLLER::getRightX));
  
      // Register named commands
      registerNamedCommands();
  
      // Bind buttons and triggers
      configureBindings();
  
      // Set up the auto builder
      DRIVE_SUBSYSTEM.configureAutoBuilder();

      // Set up the auto chooser
      automodeChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(Constants.SmartDashboardConstants.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

      // Initialize autos
      initializeAutos();

      return new RobotContainer();
  }

  private static void registerNamedCommands() {
  }

  private static void initializeAutos() {
    PathPlannerAuto leaveAuto = new PathPlannerAuto("Leave");
    //PathPlannerAuto preLoad1 = new PathPlannerAuto("Preload + 1");
    //PathPlannerAuto preLoad3 = new PathPlannerAuto("Preload + 1");
  }

  private static void configureBindings() {
    // Start - toggle traction control
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.start(), DRIVE_SUBSYSTEM.toggleTractionControlCommand(), Commands.none());

    // Left POV - Reset pose
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.povLeft(), DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new), Commands.none());

    // Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM.DRIVETRAIN_HARDWARE.navx::reset, DRIVE_SUBSYSTEM), Commands.none());

    // X - Toggle centricity
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.x(), DRIVE_SUBSYSTEM.toggleCentricityCommand(), Commands.none());

    // Right Trigger - Up
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.rightTrigger(), ELEVATOR_SUBSYSTEM.up(), ELEVATOR_SUBSYSTEM.stop());

    // Left Trigger - Down
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.leftTrigger(), ELEVATOR_SUBSYSTEM.down(), ELEVATOR_SUBSYSTEM.stop());

    // POV Down - Move Climber out
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.povDown(), DEEP_CLIMB_SUBSYSTEM.out(), DEEP_CLIMB_SUBSYSTEM.stop());
  
    // POV Up - Move Climber in
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.povUp(), DEEP_CLIMB_SUBSYSTEM.in(), DEEP_CLIMB_SUBSYSTEM.stop());

    // POV Left - Move Drawbridge up
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.povLeft(), SCORING_SUBSYSTEM.drawBridgeUp(), SCORING_SUBSYSTEM.drawBridgeStop());

    // POV Right - Move Drawbridge down
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.povRight(), SCORING_SUBSYSTEM.drawBridgeDown(), SCORING_SUBSYSTEM.drawBridgeStop());

    // A - Shoot
    RobotUtils.bindControl(HIDConstants.PRIMARY_CONTROLLER.a(), SCORING_SUBSYSTEM.outtake(), SCORING_SUBSYSTEM.outtakeStop());

  }


  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopPeriodic() {
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.CBSSubsystem;
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
  public static final CBSSubsystem COAXIAL_BOOM_STICK = new CBSSubsystem(17);


  private static SendableChooser<Command> automodeChooser; 

  public static RobotContainer createContainer(){
        // Set drive command
        // LeftY is the xRequest and LeftX is the yRequest for some reason
        DRIVE_SUBSYSTEM.setDefaultCommand(
          DRIVE_SUBSYSTEM.driveCommand(
            HIDConstants.DRIVER_CONTROLLER::getLeftY,
            HIDConstants.DRIVER_CONTROLLER::getLeftX,
            HIDConstants.DRIVER_CONTROLLER::getRightX));
  
      // Register named commands
      registerNamedCommands();
  
      // Bind buttons and triggers
      configureBindings();
  
      // Set up the auto builder
      DRIVE_SUBSYSTEM.configureAutoBuilder();

      // Set up the auto chooser
      automodeChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(Constants.SmartDashboardConstants.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

      return new RobotContainer();
  }

  private static void registerNamedCommands() {
    NamedCommands.registerCommand("Outtake", RobotUtils.timedCommand(1, SCORING_SUBSYSTEM.outtake(), SCORING_SUBSYSTEM.outtakeStop()));
    NamedCommands.registerCommand("Elevator L3", ELEVATOR_SUBSYSTEM.autoGoLevelThree());
    NamedCommands.registerCommand("Elevator L2", ELEVATOR_SUBSYSTEM.autoGoLevelTwo());
    NamedCommands.registerCommand("Elevator L1", ELEVATOR_SUBSYSTEM.autoGoLevelOne());
    NamedCommands.registerCommand("Elevator Bottom", ELEVATOR_SUBSYSTEM.autoGoToBottom());
    NamedCommands.registerCommand("Elevator Intake", ELEVATOR_SUBSYSTEM.autoGoToIntake());
    NamedCommands.registerCommand("Drawbridge Bottom", SCORING_SUBSYSTEM.goToDrawBridgeBottom());
    NamedCommands.registerCommand("Drawbridge Fullback", SCORING_SUBSYSTEM.goToDrawBridgeFullBack());
    NamedCommands.registerCommand("Intake", RobotUtils.timedCommand(0.25, SCORING_SUBSYSTEM.outtake(), SCORING_SUBSYSTEM.outtakeStop()));
  }

  private static void configureBindings() {
    // Driver Start - toggle traction control
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.start(), DRIVE_SUBSYSTEM.toggleTractionControlCommand(), Commands.none());

    // Driver Left Stick Button - Reset pose
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftStick(), DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new), Commands.none());

    // Driver Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM.DRIVETRAIN_HARDWARE.navx::reset, DRIVE_SUBSYSTEM), Commands.none());

    // Driver control for intake and outtake (No duplication with operator controller)
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightTrigger(), SCORING_SUBSYSTEM.outtake(), SCORING_SUBSYSTEM.outtakeStop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftTrigger(), SCORING_SUBSYSTEM.intake(), SCORING_SUBSYSTEM.outtakeStop());

    // Driver control for elevator (No duplication with operator controller)
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightBumper(), ELEVATOR_SUBSYSTEM.up(), ELEVATOR_SUBSYSTEM.stop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftBumper(), ELEVATOR_SUBSYSTEM.down(), ELEVATOR_SUBSYSTEM.stop());

    // Driver POV Buttons for Drawbridge and Climber
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povRight(), SCORING_SUBSYSTEM.drawBridgeUp(), SCORING_SUBSYSTEM.drawBridgeStop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povLeft(), SCORING_SUBSYSTEM.drawBridgeDown(), SCORING_SUBSYSTEM.drawBridgeStop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povUp(), DEEP_CLIMB_SUBSYSTEM.out(), DEEP_CLIMB_SUBSYSTEM.stop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povDown(), DEEP_CLIMB_SUBSYSTEM.in(), DEEP_CLIMB_SUBSYSTEM.stop());

    // Driver control for elevator levels and reset
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.y(), ELEVATOR_SUBSYSTEM.goLevelOne(), ELEVATOR_SUBSYSTEM.stop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.x(), ELEVATOR_SUBSYSTEM.goLevelTwo(), ELEVATOR_SUBSYSTEM.stop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.a(), ELEVATOR_SUBSYSTEM.goLevelThree(), ELEVATOR_SUBSYSTEM.stop());
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.b(), ELEVATOR_SUBSYSTEM.goToBottom(), ELEVATOR_SUBSYSTEM.stop());

    // Reset encoder on D-pad down
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povDown(), ELEVATOR_SUBSYSTEM.resetEncoder(), Commands.none());
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.common.LocalADStarAK;
import frc.robot.common.components.RobotContainerRegistry;
import frc.robot.common.components.RobotExceptionHandler;
import frc.robot.common.components.RobotUtils;
import org.lasarobotics.utils.GlobalConstants;

import java.nio.file.Path;

import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.interfaces.IRobotContainer;


/**
 * "Starting point" of the robot, nothing in here should need to be touched.
 * 
 * This sets up the ExceptionHandler, PathPlanner, Logging, and then creates a IRobotContainer based off the team number  
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private IRobotContainer robotContainer;
  public Robot() {
    try{
      PurpleManager.initialize( //PurpleSwerve runs this here, tho javadoc says to do it in robotInit() 
        this,
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
        Path.of("/media/sda1"),
        BuildConstants.MAVEN_NAME,
        BuildConstants.GIT_SHA,
        BuildConstants.BUILD_DATE,
        false
      );
    }
    catch (Exception e){
      System.out.println("Error loading PurpleManager" + e.getMessage() + e.getCause());
    }
  }

  @Override
  @SuppressWarnings("resource")
  public void robotInit() {

    

    Thread.setDefaultUncaughtExceptionHandler(new RobotExceptionHandler());

    // AdvantageKit Logging
    Logger.recordMetadata("ProjectName", "RAWR");
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    // Set pathfinding algorithm to be AdvantageKit compatible
    Pathfinding.setPathfinder(new LocalADStarAK());

    if (isReal()) {
        // If robot is real, log to USB drive and publish data to NetworkTables
        Logger.addDataReceiver(new WPILOGWriter("/Users/Public/Documents/FRC/Log Files/WPILogs/"));
        Logger.addDataReceiver(new NT4Publisher());
    } else {
        // Else just publish to NetworkTables for simulation or replay log file if var is set
        String replay = System.getenv(GlobalConstants.REPLAY_ENVIRONMENT_VAR);
        if (replay == null || replay.isBlank()) Logger.addDataReceiver(new NT4Publisher());
        else {
            // Run as fast as possible
            setUseTiming(false);
            // Pull the replay log from AdvantageScope (or prompt the user)
            String logPath = LogFileUtil.findReplayLog();
            // Read replay log
            Logger.setReplaySource(new WPILOGReader(logPath));
            // Save outputs to a new log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    Logger.start();
    RobotUtils.loadRobotConfig();
    robotContainer = RobotContainerRegistry.createContainerForTeam(RobotUtils.getTeamNumber());
}


  @Override
  public void robotPeriodic() {
    PurpleManager.update();
    CommandScheduler.getInstance().run();
  }

  
  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.autonomousPeriodic();
    PurpleManager.update();

  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    RobotContainer.DRIVE_SUBSYSTEM.DRIVETRAIN_HARDWARE.navx.reset();

  }

  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();
    PurpleManager.update();

  }

  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    PurpleManager.update();
  }


}

package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


/**
 * Interface for a teams RobotContainer, 
 * ideally each team that needs different functionality would implement this
 * 
 * @author Hudson Strub
 * @since 2025
 */
public interface IRobotContainer {


    RobotContainer createContainer();
    void simulationPeriodic();
    Command getAutonomousCommand();
}

package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface for a team's RobotContainer, 
 * ideally each team that needs different functionality would implement this
 * 
 * @author Hudson Strub
 * @since 2025
 */
public interface IRobotContainer {

    static IRobotContainer createContainer() {
        throw new UnsupportedOperationException("createContainer must be implemented in the specific RobotContainer class");
    }

    void simulationPeriodic();
    Command getAutonomousCommand();
}
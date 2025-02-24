package frc.robot.common.subsystems;
import com.revrobotics.spark.*;
import  com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private static final double BOTTOM_POSITION = 0.0;
    private static final double L1_POSITION =  10;
    private static final double L2_POSITION = 40.0;
    private static final double L3_POSITION = 40.0;

    private static final double TOLERANCE = 0.7; // Allowable error margin

    public ElevatorSubsystem(int motorID) {
        motor = EasyMotor.createEasySparkFlex(motorID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        encoder = motor.getEncoder();
        encoder.setPosition(0); // Reset encoder position at startup
    }

    public Command up() {
        return Commands.run(() -> motor.set(0.3), this);
    }

    public Command down() {
        return Commands.run(() -> motor.set(-0.3), this);
    }

    public Command stop() {
        return Commands.run(() -> motor.set(0.03), this);
    }

    public Command goToBottom() {
        return RobotUtils.moveToPosition(motor, BOTTOM_POSITION, TOLERANCE);
    }

    public Command goLevelOne() {
        return RobotUtils.moveToPosition(motor, L1_POSITION, TOLERANCE);
    }

    public Command goLevelTwo() {
        return RobotUtils.moveToPosition(motor, L2_POSITION, TOLERANCE);
    }

    public Command goLevelThree() {
        return RobotUtils.moveToPosition(motor, L3_POSITION, TOLERANCE);
    }
}

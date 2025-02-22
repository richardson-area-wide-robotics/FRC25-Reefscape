package frc.robot.common.subsystems;
import com.revrobotics.spark.*;
import  com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private static final double BOTTOM_POSITION = 0.0;
    private static final double L1_POSITION = 0.2;
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

    // Command to move to a specific position
    public Command moveToPosition(double targetPosition) {
        return run(() -> {
            double currentPosition = encoder.getPosition();
            double error = targetPosition - currentPosition;
            double speed = Math.copySign(0.3, error); // Move up or down based on the error
            
            if (Math.abs(error) > TOLERANCE) {
                motor.set(speed);
            } else {
                motor.set(0.0);
            }
        }).until(() -> Math.abs(encoder.getPosition() - targetPosition) < TOLERANCE);
    }

    public Command goToBottom() {
        return moveToPosition(BOTTOM_POSITION);
    }

    public Command goLevelOne() {
        return moveToPosition(L1_POSITION);
    }

    public Command goLevelTwo() {
        return moveToPosition(L2_POSITION);
    }

    public Command goLevelThree() {
        return moveToPosition(L3_POSITION);
    }
}

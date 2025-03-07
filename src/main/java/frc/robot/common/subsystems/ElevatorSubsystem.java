package frc.robot.common.subsystems;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import  com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.RobotUtils;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private static final double BOTTOM_POSITION = 0.2;
    private static final double L1_POSITION =  18.0;
    private static final double L2_POSITION = 26.0;
    private static final double L3_POSITION = 45.5;

    public ElevatorSubsystem(int motorID) {
       SparkFlexConfig config = new SparkFlexConfig();

       config.closedLoop.p(0.1).i(0).d(0).outputRange(-1, 1);

        motor = new SparkFlex(motorID, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);        
        
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
        return Commands.runOnce(()->RobotUtils.moveToPosition(motor, BOTTOM_POSITION));
    }

    public Command goLevelOne() {
        return Commands.runOnce(()-> RobotUtils.moveToPosition(motor, L1_POSITION));
    }

    public Command goLevelTwo() {
        return Commands.runOnce(()->RobotUtils.moveToPosition(motor, L2_POSITION));
    }

    public Command goLevelThree() {
        return Commands.runOnce(()->RobotUtils.moveToPosition(motor, L3_POSITION));
    }
}

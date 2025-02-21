package frc.robot.common.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;

public class DeepClimbSubsystem extends SubsystemBase {

    public SparkFlex motor;
    public SparkFlex motor2;

    public DeepClimbSubsystem(int motorID, int motor2ID) {
        motor = EasyMotor.createEasySparkFlex(motorID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        motor2 = EasyMotor.createEasySparkFlex(motor2ID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);

    }

    public Command up() {
        return Commands.run(() -> motor.set(0.3), this).andThen(() -> motor2.set(-0.3), this); //TODO This might be too fast?
    }
    public Command down() {
        return Commands.run(() -> motor.set(-0.3), this);
    }
    public Command stop(){
        return Commands.run(() -> motor.set(0), this);
    }
}

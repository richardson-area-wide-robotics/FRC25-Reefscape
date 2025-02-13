package frc.robot.common.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;

public class ElevatorSubsystem extends SubsystemBase {

    public SparkFlex motor;

    public ElevatorSubsystem(int motorID) {
        motor = EasyMotor.createEasySparkFlex(motorID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
    }

    public Command up() {
        return Commands.run(() -> motor.set(1), this); //TODO This might be too fast?
    }
    public Command down() {
        return Commands.run(() -> motor.set(-0.4), this);
    }
    public Command stop(){
        return Commands.run(() -> motor.set(0), this);
    }
}

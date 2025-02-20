package frc.robot.common.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;

public class ScoringSubsystem extends SubsystemBase {

    SparkMax drawbridgeMotor;
    SparkMax outtakeMotor;

    public ScoringSubsystem(int drawbridgeMotorId, int outtakeMotorId) {
        drawbridgeMotor = EasyMotor.createEasySparkMax(drawbridgeMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        outtakeMotor = EasyMotor.createEasySparkMax(outtakeMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
    }

    public Command drawBridgeUp() {
        return Commands.run(() -> drawbridgeMotor.set(0.3), this); //TODO This might be too fast?
    }
    public Command drawBridgeDown() {
        return Commands.run(() -> drawbridgeMotor.set(-0.3), this);
    }
    public Command drawBridgeStop(){
        return Commands.run(() -> drawbridgeMotor.set(0), this);
    }
}

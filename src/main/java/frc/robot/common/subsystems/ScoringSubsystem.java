package frc.robot.common.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;

public class ScoringSubsystem extends SubsystemBase {

    SparkFlex drawbridgeMotor;
    SparkFlex outtakeMotor;

    public ScoringSubsystem(int drawbridgeMotorId, int outtakeMotorId) {
        drawbridgeMotor = EasyMotor.createEasySparkFlex(drawbridgeMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        outtakeMotor = EasyMotor.createEasySparkFlex(outtakeMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
    }

    public Command drawBridgeUp() {
        return Commands.run(() -> drawbridgeMotor.set(0.09), this);
    }
    public Command drawBridgeDown() {
        return Commands.run(() -> drawbridgeMotor.set(-0.09), this);
    }
    public Command drawBridgeStop(){
        return Commands.run(() -> drawbridgeMotor.set(0.02), this);
    }

    public Command outtakeStop(){
        return Commands.run(() -> outtakeMotor.set(0), this);
    }

    public Command outtake(){
        return Commands.run(() -> outtakeMotor.set(0.3), this);
    }

    public Command intake(){
        return Commands.run(() -> outtakeMotor.set(-0.3), this);
    }
}
//
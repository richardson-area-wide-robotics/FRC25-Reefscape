package frc.robot.common.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;

public class ScoringSubsystem extends SubsystemBase {

    SparkFlex drawbridgeMotor;
    SparkFlex outtakeMotor;

    private static final double BOTTOM_POSITION = 0.1;
    private static final double FULLBACK_POSITION =  5.3;

    public ScoringSubsystem(int drawbridgeMotorId, int outtakeMotorId) {
       SparkFlexConfig config = new SparkFlexConfig();

       config.closedLoop.p(0.1).i(0).d(0).outputRange(-1, 1);

        drawbridgeMotor = new SparkFlex(drawbridgeMotorId, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        drawbridgeMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);        
        
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
        return Commands.runOnce(() -> outtakeMotor.set(0), this);
    }

    public Command outtake(){ 
        if (RobotUtils.getTeamNumber() == 8874){
            return Commands.run(() -> outtakeMotor.set(0.2), this);
        }
        else{
            return Commands.run(() -> outtakeMotor.set(-0.2), this);

        }

    }

    public Command intake(){
        if (RobotUtils.getTeamNumber() == 8874){
            return Commands.run(() -> outtakeMotor.set(-0.2), this);
        }
        else{
            return Commands.run(() -> outtakeMotor.set(0.2), this);

        }
    }

    public Command goToDrawBridgeBottom(){
        return Commands.runOnce(() -> RobotUtils.moveToPosition(drawbridgeMotor, BOTTOM_POSITION));
    }

    public Command goToDrawBridgeFullBack() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(drawbridgeMotor, FULLBACK_POSITION));
    }
}

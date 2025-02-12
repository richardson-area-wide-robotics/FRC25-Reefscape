package frc.robot.common.subsystems.shooter;

import frc.robot.common.components.SingleMotorSubsystem;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

public class KitBotShooter extends SingleMotorSubsystem {
    //The majority of our robot is single motor stuff without much else,
    //ideally you wouldn't even need to make a class for this

    public KitBotShooter(int motorID) {
        super(motorID);

        System.out.println("Motor: " + this.motor);
    }
}
package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KitBotShooter extends SubsystemBase {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMax motor;

    public KitBotShooter(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Set the speed of the motor.
     * @param speed The speed to set (-1.0 to 1.0).
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Stop the motor by setting its speed to 0.
     */
    public void stopMotor() {
        motor.set(0);
    }

    /**
     * Creates a command to set the shooter motor to a specific speed.
     * @param speed The speed to set (-1.0 to 1.0).
     * @return A command that sets the motor speed.
     */
    public Command setSpeedCommand(double speed) {
        return Commands.run(() -> setSpeed(speed), this);
    }

    /**
     * Creates a command to stop the shooter motor.
     * @return A command that stops the motor.
     */
    public Command stopMotorCommand() {
        return Commands.run(this::stopMotor, this);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

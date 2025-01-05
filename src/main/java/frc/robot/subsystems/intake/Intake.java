package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {

    private final CANSparkMax intakeMotor;

    @Getter @Setter
    private IntakeState m_intakeState = IntakeState.IDLE;

    // private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    // private GenericEntry m_speedEntry = intakeTab.add("Intake Speed",
    // 0).getEntry();
    // private GenericEntry m_currentEntry = intakeTab.add("Intake Current",
    // 0).getEntry();
    // private GenericEntry m_stateEntry = intakeTab.add("Intake State",
    // m_intakeState).getEntry();

    public Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCANID, CANSparkMax.MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);
        intakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);
        intakeMotor.burnFlash();

        this.getDefaultCommand();
    }

    /**
     * Commands the intake to spin in the positive direction - intaking
     */
    public Command runIntake() {
        m_intakeState = IntakeState.INTAKE;
        return Commands.run(() -> intakeMotor.set(IntakeConstants.kIntakeSpeed), this);
    }

    /**
     * Commands the intake stop
     */
    public Command runStop() {
        m_intakeState = IntakeState.IDLE;
        return Commands.run(() -> intakeMotor.set(0), this);
    }

    /**
     * Commands the intake to spin in the negative direction - outtakeing
     */
    public Command runOuttake() {
        m_intakeState = IntakeState.OUTTAKE;
        return Commands.run(() -> intakeMotor.set(IntakeConstants.kOuttakeSpeed), this);
    }

}

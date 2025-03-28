package frc.robot.common.components;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class EasyBreakBeam {
    DigitalInput breakBeam;

    public EasyBreakBeam(int channel) {
        breakBeam = new DigitalInput(channel);
    }

    public boolean getBreakBeamStatus() {
        return breakBeam.get();
    }

    public BooleanSupplier getBooleanSupplier() {
        return () -> getBreakBeamStatus();
    }

    public Command runUntilBroken(Command command, Command stopCommand) {
        return Commands.either(command, stopCommand, getBooleanSupplier());
    }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.breakbeam.Breakbeam;
import frc.robot.subsystems.breakbeam.BreakbeamIO;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Breakbeam breakbeam;

    public Intake(IntakeIO io, BreakbeamIO bbio) {
        this.io = io;
        breakbeam = new Breakbeam(bbio);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void forward() {
        io.setVoltage(4);
    }

    public void reverse() {
        io.setVoltage(-4);
    }

    public void stop() {
        io.setVoltage(0);
    }

    public boolean hasNote() {
        return breakbeam.isTripped();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
}

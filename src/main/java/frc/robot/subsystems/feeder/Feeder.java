package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.breakbeam.Breakbeam;
import frc.robot.subsystems.breakbeam.BreakbeamIO;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final Breakbeam breakbeam;

  public Feeder(FeederIO io, BreakbeamIO bbio) {
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

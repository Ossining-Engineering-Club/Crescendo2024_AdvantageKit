package frc.robot.subsystems.breakbeam;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Breakbeam extends SubsystemBase {
  private final double breakbeamVoltageThreshold = 2.5;
  private final int samplingWindow = 2;

  private int onCount = 0;
  private final BreakbeamIO io;
  private final BreakbeamIOInputsAutoLogged inputs = new BreakbeamIOInputsAutoLogged();

  public Breakbeam(BreakbeamIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (inputs.breakbeamVoltage < breakbeamVoltageThreshold) onCount++;
    else onCount = 0;
  }

  public boolean isTripped() {
    return onCount >= samplingWindow;
  }
}

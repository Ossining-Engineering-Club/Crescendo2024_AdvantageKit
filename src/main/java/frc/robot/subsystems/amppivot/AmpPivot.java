package frc.robot.subsystems.amppivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AmpPivot extends SubsystemBase {
  private final AmpPivotIO io;
  private final AmpPivotIOInputsAutoLogged inputs = new AmpPivotIOInputsAutoLogged();

  private final PIDController pid;

  public AmpPivot(AmpPivotIO io) {
    this.io = io;

    pid = new PIDController(0.0055, 0, 0);
    pid.setTolerance(AmpPivotConstants.kPIDTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AmpPivot", inputs);
  }

  public double getAngle() {
    return inputs.angle;
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public void runSetpoint(double setpoint) {
    io.setVoltage(pid.calculate(getAngle(), setpoint));
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}

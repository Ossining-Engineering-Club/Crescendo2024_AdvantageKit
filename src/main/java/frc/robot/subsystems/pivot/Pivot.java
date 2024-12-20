package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final PIDController pid;
  private final ArmFeedforward ffModel;

  public Pivot(PivotIO io) {
    this.io = io;

    pid = new PIDController(.85, 0, 0);
    ffModel = new ArmFeedforward(0, -0.026, 0);

    pid.setTolerance(PivotConstants.kPIDTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  public double getAngle() {
    return inputs.angle;
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public boolean atAmpAngle() {
    return Math.abs(getAngle() - PivotConstants.kAmpAngle)
        <= PivotConstants.kAmpAngleCheckingTolerance;
  }

  public void runSetpoint(double setpoint) {
    io.setVoltage(
        pid.calculate(getAngle(), setpoint)
            + ffModel.calculate(Units.degreesToRadians(setpoint), 0.0));
  }

  public void stop() {
    io.setVoltage(0);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}

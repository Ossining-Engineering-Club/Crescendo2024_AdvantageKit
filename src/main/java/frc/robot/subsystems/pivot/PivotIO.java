package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double appliedVolts = 0.0;
    public double angle = 45.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run the intake motor at the specified voltage. */
  public default void setVoltage(double voltage) {}
}

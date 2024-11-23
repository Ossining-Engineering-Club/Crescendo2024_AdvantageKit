package frc.robot.subsystems.amppivot;

import org.littletonrobotics.junction.AutoLog;

public interface AmpPivotIO {
  @AutoLog
  public static class AmpPivotIOInputs {
    public double appliedVolts = 0.0;
    public double angle = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AmpPivotIOInputs inputs) {}

  /** Run the intake motor at the specified voltage. */
  public default void setVoltage(double voltage) {}
}

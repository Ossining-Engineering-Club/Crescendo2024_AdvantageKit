package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake motor at the specified voltage. */
  public default void setVoltage(double voltage) {}
}

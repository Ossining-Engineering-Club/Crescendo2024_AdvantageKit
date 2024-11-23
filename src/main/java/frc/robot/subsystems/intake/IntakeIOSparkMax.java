package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax intakeSparkMax;

  public IntakeIOSparkMax() {
    intakeSparkMax = new CANSparkMax(21, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = intakeSparkMax.getAppliedOutput() * intakeSparkMax.getBusVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    intakeSparkMax.setVoltage(voltage);
  }
}

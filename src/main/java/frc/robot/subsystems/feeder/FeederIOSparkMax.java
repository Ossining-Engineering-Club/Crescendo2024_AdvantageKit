package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class FeederIOSparkMax implements FeederIO {
  private final CANSparkMax feederSparkMax;

  public FeederIOSparkMax() {
    feederSparkMax = new CANSparkMax(33, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.appliedVolts = feederSparkMax.getAppliedOutput() * feederSparkMax.getBusVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    feederSparkMax.setVoltage(voltage);
  }
}

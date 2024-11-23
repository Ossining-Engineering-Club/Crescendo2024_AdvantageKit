package frc.robot.subsystems.amppivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class AmpPivotIOSparkMax implements AmpPivotIO {
  private final double kAngleGearRatio = 1.0 / 40.0 * (22.0 / 32.0) * 360.0;

  private final CANSparkMax ampPivotSparkMax;
  private final RelativeEncoder angleEncoder;

  public AmpPivotIOSparkMax() {
    ampPivotSparkMax = new CANSparkMax(35, MotorType.kBrushless);
    angleEncoder = ampPivotSparkMax.getEncoder();

    angleEncoder.setPositionConversionFactor(kAngleGearRatio);
    angleEncoder.setVelocityConversionFactor(kAngleGearRatio * 60);
    angleEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(AmpPivotIOInputs inputs) {
    inputs.appliedVolts = ampPivotSparkMax.getAppliedOutput() * ampPivotSparkMax.getBusVoltage();
    inputs.angle = angleEncoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    ampPivotSparkMax.setVoltage(voltage);
  }
}

package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOSparkMax implements PivotIO {
    private final double kAbsEncoderOffset = 20.51;
    private final double kAngleEncoderGearRatio = 15.0 / 46.0 * 360.0;
    private final boolean kIsAngleMotorInverted = true;

    private final CANSparkMax pivotSparkMax;
    private final DutyCycleEncoder angleEncoder;

    public PivotIOSparkMax() {
        pivotSparkMax = new CANSparkMax(32, MotorType.kBrushless);
        angleEncoder = new DutyCycleEncoder(0);

        pivotSparkMax.setInverted(kIsAngleMotorInverted);
        angleEncoder.setDistancePerRotation(kAngleEncoderGearRatio);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVolts = pivotSparkMax.getAppliedOutput() * pivotSparkMax.getBusVoltage();
        inputs.angle = angleEncoder.getDistance()-kAbsEncoderOffset;
    }

    @Override
    public void setVoltage(double voltage) {
        pivotSparkMax.setVoltage(voltage);
    }
}

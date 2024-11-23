package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amppivot.AmpPivot;

public class AngleAmpPivot extends Command {
  private final AmpPivot ampPivot;
  private final double angle;

  public AngleAmpPivot(double angle, AmpPivot ampPivot) {
    this.ampPivot = ampPivot;
    this.angle = angle;

    addRequirements(ampPivot);
  }

  @Override
  public void execute() {
    ampPivot.runSetpoint(angle);
  }

  @Override
  public void end(boolean interrupted) {
    ampPivot.stop();
  }

  @Override
  public boolean isFinished() {
    return ampPivot.atSetpoint();
  }
}

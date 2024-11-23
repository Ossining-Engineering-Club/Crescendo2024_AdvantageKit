package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class IntakeNoteToShooter extends Command {
  private final Intake intake;
  private final Feeder feeder;

  public IntakeNoteToShooter(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;

    addRequirements(intake, feeder);
  }

  @Override
  public void initialize() {
    intake.forward();
    feeder.forward();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return feeder.hasNote();
  }
}

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FeedToFlywheels extends Command {
  private final Feeder feeder;

  public FeedToFlywheels(Feeder feeder) {
    this.feeder = feeder;
  }

  @Override
  public void initialize() {
    feeder.forward();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return !feeder.hasNote();
  }
}

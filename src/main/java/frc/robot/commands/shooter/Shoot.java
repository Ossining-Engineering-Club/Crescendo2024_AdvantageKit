package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;

public class Shoot extends SequentialCommandGroup {
  public Shoot(double RPM, Flywheel flywheel, Feeder feeder) {
    addCommands(
        Commands.runOnce(() -> flywheel.runVelocity(RPM)),
        new FeedToFlywheels(feeder),
        new WaitCommand(0.25),
        Commands.runOnce(() -> flywheel.stop()));
  }
}

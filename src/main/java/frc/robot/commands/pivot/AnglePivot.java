package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class AnglePivot extends Command {
    private final Pivot pivot;
    private final double angle;
    
    public AnglePivot(double angle, Pivot pivot) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.runSetpoint(angle);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return pivot.atSetpoint();
    }
}

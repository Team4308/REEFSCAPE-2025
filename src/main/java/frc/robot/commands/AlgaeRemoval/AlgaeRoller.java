
package frc.robot.commands.AlgaeRemoval;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollerSubsystem;

public class AlgaeRoller extends Command {
    private final CoralRollerSubsystem subsystem;
    private final Supplier<Double> control;

    public AlgaeRoller(Supplier<Double> control, CoralRollerSubsystem rollerSubsystem) {
        this.subsystem = rollerSubsystem;
        this.control = control;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setRollerOutput(control.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopControllers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

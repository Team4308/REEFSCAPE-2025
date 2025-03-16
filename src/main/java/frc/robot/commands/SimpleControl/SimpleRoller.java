package frc.robot.commands.SimpleControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollerSubsystem;

public class SimpleRoller extends Command {
    private final Supplier<Double> control;
    private final CoralRollerSubsystem subsystem;

    public SimpleRoller(Supplier<Double> control, CoralRollerSubsystem rollerSubsystem) {
        this.control = control;
        this.subsystem = rollerSubsystem;
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
        return (subsystem.beamBreak.get());
    }

}

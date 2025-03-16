package frc.robot.commands.SimpleControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SimpleElevator extends Command {
    private final Supplier<Double> control;
    private final ElevatorSubsystem subsystem;

    public SimpleElevator(Supplier<Double> control, ElevatorSubsystem elevatorSubsystem) {
        this.control = control;
        this.subsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setPosition(control.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopControllers();
    }

    @Override
    public boolean isFinished() {
        return (subsystem.isAtPosition());
    }

}

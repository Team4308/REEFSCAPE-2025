
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualControlElevator extends Command {
    private final Supplier<Double> control;
    private final ElevatorSubsystem subsystem;

    public ManualControlElevator(Supplier<Double> control, ElevatorSubsystem elevatorSubsystem) {
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
        subsystem.setPositionCommand(control.get());
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

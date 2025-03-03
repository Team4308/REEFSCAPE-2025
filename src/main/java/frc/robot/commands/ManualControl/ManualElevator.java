package frc.robot.commands.ManualControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevator extends Command {
    private final Supplier<Double> angle;
    private final ElevatorSubsystem subsystem;

    public ManualElevator(Supplier<Double> angle, ElevatorSubsystem elevatorSubsystem) {
        this.angle = angle;
        this.subsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setPosition(subsystem.getPositionInMeters() + angle.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopControllers();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isAtPosition();
    }

}

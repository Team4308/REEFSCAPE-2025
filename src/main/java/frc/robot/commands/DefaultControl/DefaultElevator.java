package frc.robot.commands.DefaultControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevator extends Command {
    private final Supplier<Double> angle;
    private final ElevatorSubsystem subsystem;

    public DefaultElevator(Supplier<Double> angle, ElevatorSubsystem elevatorSubsystem) {
        this.angle = angle;
        this.subsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        subsystem.setPosition(subsystem.targetPosition + angle.get());
    }
}
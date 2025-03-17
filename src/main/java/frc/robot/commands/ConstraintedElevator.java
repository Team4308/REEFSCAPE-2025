package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ConstraintedElevator extends Command {
    private final Supplier<Double> control;
    private final Supplier<Double> maxVelocity;
    private final Supplier<Double> maxAcceleration;
    private final ElevatorSubsystem subsystem;

    private double oldAcceleration;
    private double oldVelocity;

    public ConstraintedElevator(Supplier<Double> control, Supplier<Double> maxVelocity,
            Supplier<Double> maxAcceleration, ElevatorSubsystem elevatorSubsystem) {
        this.control = control;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.subsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        Pair<Double, Double> curvalue = subsystem.getConstraints();
        oldVelocity = curvalue.getFirst();
        oldAcceleration = curvalue.getSecond();
        subsystem.setConstraints(maxVelocity.get(), maxAcceleration.get());
    }

    @Override
    public void execute() {
        subsystem.setPosition(control.get());
    }

    @Override
    public void end(boolean whatDoesThisDo) {
        subsystem.setConstraints(oldVelocity, oldAcceleration);
    }

    @Override
    public boolean isFinished() {
        return (subsystem.isAtPosition());
    }

}

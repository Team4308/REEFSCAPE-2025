
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorwithVelocity extends Command {
    private final Supplier<Double> targetHeight;
    private final Supplier<Double> targetVelocity;
    private final ElevatorSubsystem subsystem;
    private double beginningSpeed;

    public ElevatorwithVelocity(Supplier<Double> targetHeight, Supplier<Double> targetVelocity,
            ElevatorSubsystem elevatorSubsystem) {
        this.targetHeight = targetHeight;
        this.targetVelocity = targetVelocity;
        this.subsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
        beginningSpeed = subsystem.getCurrentSpeed();
        subsystem.setCustomSpeed(targetVelocity.get());
    }

    @Override
    public void execute() {
        subsystem.setPosition(targetHeight.get());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopControllers();
        subsystem.setCustomSpeed(beginningSpeed);
    }

    @Override
    public boolean isFinished() {
        return subsystem.isAtPosition();
    }

}

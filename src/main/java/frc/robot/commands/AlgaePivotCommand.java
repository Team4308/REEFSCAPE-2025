package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivotSubsystem;

public class AlgaePivotCommand extends Command {
    private final Supplier<Double> control;
    private final AlgaePivotSubsystem subsystem;

    public AlgaePivotCommand(Supplier<Double> control, AlgaePivotSubsystem algaePivotSubsystem) {
        this.control = control; 
        this.subsystem = algaePivotSubsystem;
        addRequirements(algaePivotSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setOutput(control.get());
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
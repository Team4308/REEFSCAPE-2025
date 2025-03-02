package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class ManualControlAlgae extends Command {
    private final Supplier<Double> control;
    private final AlgaeArmSubsystem subsystem;

    public ManualControlAlgae(Supplier<Double> control, AlgaeArmSubsystem algaePivotSubsystem) {
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
        subsystem.setAlgaePosition(control.get());
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

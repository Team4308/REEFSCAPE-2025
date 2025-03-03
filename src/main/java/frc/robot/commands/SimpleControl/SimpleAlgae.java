package frc.robot.commands.SimpleControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class SimpleAlgae extends Command {
    private final Supplier<Double> control;
    private final AlgaeArmSubsystem subsystem;

    public SimpleAlgae(Supplier<Double> control, AlgaeArmSubsystem algaePivotSubsystem) {
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
        return subsystem.isAtPosition();
    }
    
}

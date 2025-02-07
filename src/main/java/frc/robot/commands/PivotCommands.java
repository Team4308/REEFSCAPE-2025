package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotCommands extends Command {
    
    private final PivotSubsystem m_subsystem;
    private final Supplier<Double> control;

    // init
    public PivotCommands(PivotSubsystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control; 
        addRequirements(m_subsystem);

    }

    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    @Override
    public void execute() {
        double control = this.control.get();
        m_subsystem.setMotorPosition(control);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
}



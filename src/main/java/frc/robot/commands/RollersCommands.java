package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollersSubsystem;

public class RollersCommands extends Command {
    
    private final RollersSubsystem m_subsystem;
    private final Supplier<Double> control;

    // Init
    public RollersCommands(RollersSubsystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
        @Override
    public void execute() {
        double control = this.control.get();
        m_subsystem.setMotorOutput(control);
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

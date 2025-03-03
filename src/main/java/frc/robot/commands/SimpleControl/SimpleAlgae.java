package frc.robot.commands.SimpleControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class SimpleAlgae extends Command {
    private final Supplier<Double> angle;
    private final AlgaeArmSubsystem subsystem;

    public SimpleAlgae(Supplier<Double> angle, AlgaeArmSubsystem algaePivotSubsystem) {
        this.angle = angle;
        this.subsystem = algaePivotSubsystem;
        addRequirements(algaePivotSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setAlgaePosition(angle.get());
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

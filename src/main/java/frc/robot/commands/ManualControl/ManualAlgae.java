package frc.robot.commands.ManualControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class ManualAlgae extends Command {
    private final Supplier<Double> angle;
    private final AlgaeArmSubsystem subsystem;

    public ManualAlgae(Supplier<Double> angle, AlgaeArmSubsystem algaePivotSubsystem) {
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
        subsystem.setAlgaePosition(subsystem.targetAngle + angle.get());
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
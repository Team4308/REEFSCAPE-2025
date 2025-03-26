package frc.robot.commands.SimpleControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class SimpleAlgae extends Command {
    private final Supplier<Double> angle;
    private final AlgaeArmSubsystem subsystem;

    private double startTime;

    public SimpleAlgae(Supplier<Double> angle, AlgaeArmSubsystem algaePivotSubsystem) {
        this.angle = angle;
        this.subsystem = algaePivotSubsystem;
        addRequirements(algaePivotSubsystem);

        startTime = Timer.getTimestamp();
    }

    @Override
    public void initialize() {
        subsystem.setAlgaePosition(angle.get());
    }

    @Override
    public boolean isFinished() {
        return (subsystem.isAtPosition2(angle.get())) || Math.abs(Timer.getTimestamp() - startTime) > 3;
    }

}

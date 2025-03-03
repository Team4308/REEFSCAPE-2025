
package frc.robot.commands.AlgaeRemoval;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralRollerSubsystem;

public class AlgaeRoller extends Command {
    private final CoralRollerSubsystem subsystem;

    public AlgaeRoller(CoralRollerSubsystem rollerSubsystem) {
        this.subsystem = rollerSubsystem;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setRollerOutput(-Constants.EndEffector.speeds.removeAlgae);
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

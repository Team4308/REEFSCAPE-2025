
package frc.robot.commands.AlgaeRemoval;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralRollerSubsystem;

public class AlgaeRoller extends Command {
    private final CoralRollerSubsystem subsystem;
    private final boolean reversed;

    public AlgaeRoller(boolean reversed, CoralRollerSubsystem rollerSubsystem) {
        this.subsystem = rollerSubsystem;
        this.reversed = reversed;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        double speed = -Constants.EndEffector.speeds.removeAlgae;
        if (reversed) {
            speed *= -1;
        }
        subsystem.setRollerOutput(speed);
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

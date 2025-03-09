
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralRollerSubsystem;

public class Intake extends Command {
    private final CoralRollerSubsystem subsystem;

    public Intake(CoralRollerSubsystem rollerSubsystem) {
        this.subsystem = rollerSubsystem;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopControllers();
    }

    @Override
    public void execute() {
        subsystem.setRollerOutput(Constants.constEndEffector.speeds.intake);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopControllers();
    }

    @Override
    public boolean isFinished() {
        return !subsystem.getBeamBreak();
    }

}

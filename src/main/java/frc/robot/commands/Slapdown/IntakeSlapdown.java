
package frc.robot.commands.Slapdown;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Slapdown;
import frc.robot.subsystems.SlapdownSubsystem;

public class IntakeSlapdown extends SequentialCommandGroup {
    public IntakeSlapdown(SlapdownSubsystem subsystem) {
        addCommands(
                new SlapdownSubsystem().setPosition(Slapdown.PIVOT_BOTTOM_ANGLE),
                new SlapdownSubsystem().setIntake(Slapdown.INTAKE_SPEED));
    }
}

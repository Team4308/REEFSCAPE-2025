
package frc.robot.commands.Slapdown;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constSlapdown;
import frc.robot.subsystems.SlapdownSubsystem;

public class IntakeSlapdown extends SequentialCommandGroup {
    public IntakeSlapdown(SlapdownSubsystem subsystem) {
        addCommands(
                new SlapdownSubsystem().setPosition(constSlapdown.PIVOT_BOTTOM_ANGLE),
                new SlapdownSubsystem().setIntake(constSlapdown.INTAKE_SPEED));
    }
}

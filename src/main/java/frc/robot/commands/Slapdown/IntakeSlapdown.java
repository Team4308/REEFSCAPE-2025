
package frc.robot.commands.Slapdown;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SlapdownSubsystem;

public class IntakeSlapdown extends SequentialCommandGroup {
    public IntakeSlapdown(SlapdownSubsystem subsystem) {
        addCommands(
                new SlapdownSubsystem().setPosition(45),
                new SlapdownSubsystem().setIntake(5));
    }
}

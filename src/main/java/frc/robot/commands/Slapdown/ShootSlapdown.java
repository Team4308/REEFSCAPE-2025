
package frc.robot.commands.Slapdown;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SlapdownSubsystem;

public class ShootSlapdown extends SequentialCommandGroup {
    public ShootSlapdown(SlapdownSubsystem subsystem) {
        addCommands(
                new SlapdownSubsystem().setPosition(90),
                new SlapdownSubsystem().setIntake(-5));
    }
}

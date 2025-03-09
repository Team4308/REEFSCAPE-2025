
package frc.robot.commands.Slapdown;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Slapdown;
import frc.robot.subsystems.SlapdownSubsystem;

public class ShootSlapdown extends SequentialCommandGroup {
    public ShootSlapdown(SlapdownSubsystem subsystem) {
        addCommands(
                new SlapdownSubsystem().setPosition(Slapdown.PIVOT_TOP_ANGLE),
                new SlapdownSubsystem().setIntake(-Slapdown.INTAKE_SPEED));
    }
}

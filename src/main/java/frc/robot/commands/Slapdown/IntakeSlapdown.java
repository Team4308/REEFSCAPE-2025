
package frc.robot.commands.Slapdown;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SlapdownSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Slapdown;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class IntakeSlapdown extends SequentialCommandGroup {
    public IntakeSlapdown(SlapdownSubsystem subsystem) {
        addCommands(
                new SlapdownSubsystem().setPosition(45),
                new SlapdownSubsystem().setIntake(5));
    }
}

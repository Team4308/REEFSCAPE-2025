
package frc.robot.commands.AlgaeRemoval;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.ElevatorwithVelocity;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class RemoveL1 extends SequentialCommandGroup {
    public RemoveL1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem, AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
            new SimpleAlgae(() -> Constants.EndEffector.algaePositions.restPosition, algaeArmSubsystem),
            new SimpleElevator(() -> Constants.constElevator.ALGAE1, elevatorSubsystem),
            new ParallelDeadlineGroup(
                new ElevatorwithVelocity(() -> Constants.constElevator.MIN_HEIGHT, () -> Constants.constElevator.MIN_HEIGHT, elevatorSubsystem),
                new AlgaeRoller(rollerSubsystem)
            )
        );
    }    
}

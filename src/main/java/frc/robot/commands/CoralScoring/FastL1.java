
package frc.robot.commands.CoralScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.ElevatorwithVelocity;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class FastL1 extends SequentialCommandGroup {
    public FastL1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem, AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
            new SimpleAlgae(() -> Constants.EndEffector.algaePositions.restPosition, algaeArmSubsystem),
            new ParallelCommandGroup(
                new ElevatorwithVelocity(() -> Constants.constElevator.L1, () -> Constants.constElevator.L1Velocity, elevatorSubsystem),
                new SimpleRoller(() -> Constants.EndEffector.speeds.L1, rollerSubsystem)
            ),
            new SimpleElevator(() -> Constants.constElevator.MIN_HEIGHT, elevatorSubsystem)
        );
    }    
}

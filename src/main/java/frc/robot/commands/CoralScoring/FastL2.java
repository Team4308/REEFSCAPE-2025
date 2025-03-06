
package frc.robot.commands.CoralScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class FastL2 extends SequentialCommandGroup {
    public FastL2(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                //new SimpleAlgae(() -> Constants.EndEffector.algaePositions.restPosition, algaeArmSubsystem),
                new SimpleElevator(() -> Constants.constElevator.L2, elevatorSubsystem),
                new SimpleRoller(() -> Constants.EndEffector.speeds.L23, rollerSubsystem),
                new SimpleElevator(() -> Constants.constElevator.MIN_HEIGHT, elevatorSubsystem));
    }
}

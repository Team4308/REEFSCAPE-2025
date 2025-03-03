package frc.robot.commands.Barge;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class ShootBarge extends SequentialCommandGroup {
    public ShootBarge(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                new SimpleAlgae(() -> Constants.EndEffector.algaePositions.holdBargePosition, algaeArmSubsystem),
                new SimpleElevator(() -> Constants.constElevator.MAX_HEIGHT, elevatorSubsystem),
                new SimpleAlgae(() -> 0.0, algaeArmSubsystem),
                new SimpleAlgae(() -> 90.0, algaeArmSubsystem));
    }
}

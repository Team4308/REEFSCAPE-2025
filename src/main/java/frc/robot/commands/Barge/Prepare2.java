package frc.robot.commands.Barge;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.ElevatorwithVelocity;
import frc.robot.commands.AlgaeRemoval.AlgaeRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class Prepare2 extends SequentialCommandGroup {
        public Prepare2(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
                        AlgaeArmSubsystem algaeArmSubsystem) {
                addCommands(
                                new SimpleAlgae(() -> Constants.EndEffector.algaePositions.removeBargePosition,
                                                algaeArmSubsystem),
                                new SimpleElevator(() -> Constants.constElevator.ALGAE2 - 0.5, elevatorSubsystem),
                                new ParallelDeadlineGroup(
                                                new ElevatorwithVelocity(() -> Constants.constElevator.ALGAE2,
                                                                () -> Constants.constElevator.ALGAE_REMOVAL_SPEED,
                                                                elevatorSubsystem),
                                                new AlgaeRoller(true, rollerSubsystem)),
                                new SimpleAlgae(() -> Constants.EndEffector.algaePositions.holdBargePosition,
                                                algaeArmSubsystem));
        }
}

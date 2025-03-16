package frc.robot.commands.AlgaeRemoval;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class RemoveL2 extends SequentialCommandGroup {
        public RemoveL2(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
                        AlgaeArmSubsystem algaeArmSubsystem) {
                addCommands(
                                new SimpleElevator(() -> constElevator.ALGAE2, elevatorSubsystem),
                                new SimpleAlgae(() -> constEndEffector.algaePivot.REMOVAL_ANGLE, algaeArmSubsystem),
                                new InstantCommand(() -> elevatorSubsystem.setConstraints(
                                                constElevator.ALGAE_REMOVAL_SPEED, constElevator.MAX_ACCELERATION)),
                                new ParallelDeadlineGroup(
                                                new SimpleElevator(() -> constElevator.MIN_HEIGHT, elevatorSubsystem),
                                                new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL,
                                                                rollerSubsystem)),
                                new InstantCommand(() -> elevatorSubsystem.setConstraints(constElevator.MAX_VELOCITY,
                                                constElevator.MAX_ACCELERATION)),
                                new SimpleAlgae(() -> constEndEffector.algaePivot.MAX_ANGLE, algaeArmSubsystem));
        }
}

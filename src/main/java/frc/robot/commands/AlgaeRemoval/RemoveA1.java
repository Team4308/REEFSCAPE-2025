package frc.robot.commands.AlgaeRemoval;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgaeTimeout;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class RemoveA1 extends SequentialCommandGroup {
        public RemoveA1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
                        AlgaeArmSubsystem algaeArmSubsystem) {
                addCommands(
                                new SimpleAlgaeTimeout(() -> constEndEffector.algaePivot.REST_ANGLE, 1,
                                                algaeArmSubsystem),
                                new SimpleElevator(() -> constElevator.ALGAE1, elevatorSubsystem),
                                new SimpleAlgaeTimeout(() -> constEndEffector.algaePivot.REMOVAL_ANGLE_TOP, 1,
                                                algaeArmSubsystem),
                                new InstantCommand(() -> elevatorSubsystem.setConstraints(
                                                constElevator.ALGAE_REMOVAL_SPEED, constElevator.MAX_ACCELERATION)),
                                new ParallelDeadlineGroup(
                                                new SimpleElevator(() -> constElevator.MIN_HEIGHT, elevatorSubsystem),
                                                new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL_TOP,
                                                                rollerSubsystem)),
                                new InstantCommand(() -> elevatorSubsystem.setConstraints(constElevator.MAX_VELOCITY,
                                                constElevator.MAX_ACCELERATION)),
                                new SimpleAlgaeTimeout(() -> constEndEffector.algaePivot.REST_ANGLE, 1,
                                                algaeArmSubsystem));
        }
}

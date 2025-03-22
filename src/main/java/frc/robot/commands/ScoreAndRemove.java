package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class ScoreAndRemove extends SequentialCommandGroup {
        public ScoreAndRemove(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
                        AlgaeArmSubsystem algaeArmSubsystem) {
                addCommands(
                                new SimpleAlgae(() -> constEndEffector.algaePivot.REMOVAL_ANGLE_BOTTOM,
                                                algaeArmSubsystem),
                                new SimpleElevator(() -> constElevator.L2, elevatorSubsystem),
                                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L23, rollerSubsystem),
                                new ParallelDeadlineGroup(
                                                new SimpleElevator(() -> constElevator.ALGAE1, elevatorSubsystem),
                                                new DefaultRoller(() -> -constEndEffector.rollerSpeeds.ALGAE_REMOVAL,
                                                                rollerSubsystem)),
                                new SimpleAlgae(() -> constEndEffector.algaePivot.REST_ANGLE, algaeArmSubsystem));
        }
}

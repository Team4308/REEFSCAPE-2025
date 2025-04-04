package frc.robot.commands;

import org.ejml.dense.row.decomposition.eig.watched.WatchedDoubleStepQREigen_FDRM;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.ButtonBindings.Algae1PreMove;
import frc.robot.commands.ButtonBindings.Algae2PreMove;
import frc.robot.commands.ButtonBindings.L2Algae1;
import frc.robot.commands.ButtonBindings.L2Algae2;
import frc.robot.commands.ButtonBindings.L2PreMove;
import frc.robot.commands.ButtonBindings.L3Algae1;
import frc.robot.commands.ButtonBindings.L3Algae2;
import frc.robot.commands.ButtonBindings.L3PreMove;
import frc.robot.commands.SimpleControl.SimpleAlgaeTimeout;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class Reset extends ParallelDeadlineGroup {
        public Reset(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
                        AlgaeArmSubsystem algaeArmSubsystem) {
                super(new WaitCommand(0.02));
                addCommands(
                                new InstantCommand(() -> elevatorSubsystem.setConstraints(constElevator.MAX_VELOCITY,
                                                constElevator.MAX_ACCELERATION)),
                                new SimpleAlgaeTimeout(() -> constEndEffector.algaePivot.REST_ANGLE, 0.02,
                                                algaeArmSubsystem),
                                new SimpleElevator(() -> constElevator.MIN_HEIGHT, elevatorSubsystem)
                                                .withDeadline(new WaitCommand(0.02)),
                                new SimpleRoller(() -> 0.0, rollerSubsystem).withDeadline(new WaitCommand(0.02)),
                                new InstantCommand(() -> L2PreMove.resetCommand()),
                                new InstantCommand(() -> L3PreMove.resetCommand()),
                                new InstantCommand(() -> Algae1PreMove.resetCommand()),
                                new InstantCommand(() -> Algae2PreMove.resetCommand()),
                                new InstantCommand(() -> L2Algae1.resetCommand()),
                                new InstantCommand(() -> L2Algae2.resetCommand()),
                                new InstantCommand(() -> L3Algae1.resetCommand()),
                                new InstantCommand(() -> L3Algae2.resetCommand()));
        }
}

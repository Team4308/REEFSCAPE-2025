package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Reset;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class L3A1Stage2 extends SequentialCommandGroup {
    public L3A1Stage2(ElevatorSubsystem m_elevatorSubsystem, CoralRollerSubsystem m_coralRollerSubsystem,
            AlgaeArmSubsystem m_algaeArmSubsystem) {
        addCommands(
                new Reset(m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem),
                new SimpleRoller(() -> -constEndEffector.rollerSpeeds.DEFAULT_CORAL, m_coralRollerSubsystem),
                new ParallelDeadlineGroup(
                        new SimpleAlgae(() -> 90.0, m_algaeArmSubsystem),
                        new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL_BOTTOM,
                                m_coralRollerSubsystem)),
                new SimpleAlgae(() -> 0.0,
                        m_algaeArmSubsystem),
                new IntakeCommand(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL, m_coralRollerSubsystem),
                new InstantCommand(() -> m_elevatorSubsystem.setConstraints(constElevator.MAX_VELOCITY,
                        constElevator.MAX_ACCELERATION)),
                new SimpleAlgae(() -> constEndEffector.algaePivot.REST_ANGLE, m_algaeArmSubsystem)
                        .withDeadline(new WaitCommand(0.02)),
                new SimpleElevator(() -> constElevator.L3, m_elevatorSubsystem),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L23, m_coralRollerSubsystem),
                new Reset(m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem));
    }
}

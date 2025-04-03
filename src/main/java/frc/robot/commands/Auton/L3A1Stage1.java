package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.Reset;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class L3A1Stage1 extends SequentialCommandGroup {
    public L3A1Stage1(ElevatorSubsystem m_elevatorSubsystem, CoralRollerSubsystem m_rollerSubsystem,
            AlgaeArmSubsystem m_algaeArmSubsystem) {
        addCommands(
                new Reset(m_elevatorSubsystem, m_rollerSubsystem, m_algaeArmSubsystem),
                new ParallelCommandGroup(
                        new SimpleElevator(() -> constElevator.ALGAE1_PREMOVE, m_elevatorSubsystem),
                        new SimpleAlgae(() -> constEndEffector.algaePivot.REMOVAL_ANGLE_BOTTOM, m_algaeArmSubsystem)));
    }
}

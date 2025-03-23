package frc.robot.commands.ButtonBindings;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.Reset;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class L3Algae1 extends Command {
    private ElevatorSubsystem m_elevatorSubsystem;
    private CoralRollerSubsystem m_coralRollerSubsystem;
    private AlgaeArmSubsystem m_algaeArmSubsystem;

    public L3Algae1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_algaeArmSubsystem = algaeArmSubsystem;
        this.m_coralRollerSubsystem = rollerSubsystem;
    }

    @Override
    public void execute() {
        if (m_elevatorSubsystem.isAtPosition("A1P")) {
            Commands.run(() -> stage2(), m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem);
        } else {
            Commands.run(() -> stage1(), m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem);
        }
    }

    private Command stage1() {
        return new SequentialCommandGroup(
                new Reset(m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem),
                new ParallelCommandGroup(
                        new SimpleElevator(() -> constElevator.ALGAE1_PREMOVE, m_elevatorSubsystem),
                        new SimpleAlgae(() -> constEndEffector.algaePivot.REMOVAL_ANGLE_BOTTOM, m_algaeArmSubsystem)));
    }

    private Command stage2() {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL_BOTTOM,
                                m_coralRollerSubsystem),
                        new SimpleElevator(() -> constElevator.L3, m_elevatorSubsystem)),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L23, m_coralRollerSubsystem),
                new Reset(m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem));
    }

    @Override
    public boolean isFinished() {
        if (m_elevatorSubsystem.isAtPosition("A1P")) {
            return m_elevatorSubsystem.isAtPosition("MIN");
        } else {
            return m_elevatorSubsystem.isAtPosition("A1P");
        }
    }
}

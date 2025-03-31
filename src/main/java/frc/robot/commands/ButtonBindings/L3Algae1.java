package frc.robot.commands.ButtonBindings;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class L3Algae1 extends Command {
    private ElevatorSubsystem m_elevatorSubsystem;
    private CoralRollerSubsystem m_coralRollerSubsystem;
    private AlgaeArmSubsystem m_algaeArmSubsystem;

    private static boolean stateFinished = false;

    public L3Algae1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_algaeArmSubsystem = algaeArmSubsystem;
        this.m_coralRollerSubsystem = rollerSubsystem;
    }

    @Override
    public void initialize() {
        if (m_elevatorSubsystem.isAtPosition2("L2")) {
            stage2().schedule();
        } else {
            stage1().schedule();
        }
    }

    public static void resetCommand() {
        stateFinished = true;
    }

    private Command stage1() {
        return new SequentialCommandGroup(
                new Reset(m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem),
                new ParallelCommandGroup(
                        new SimpleElevator(() -> constElevator.L2, m_elevatorSubsystem),
                        new SimpleAlgae(() -> constEndEffector.algaePivot.REMOVAL_ANGLE_TOP, m_algaeArmSubsystem)));
    }

    private Command stage2() {
        return new SequentialCommandGroup(
                new SimpleElevator(() -> constElevator.L3, m_elevatorSubsystem),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL, m_coralRollerSubsystem),
                new ParallelDeadlineGroup(
                        new SimpleElevator(() -> constElevator.ALGAE1, m_elevatorSubsystem),
                        new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL_TOP,
                                m_coralRollerSubsystem)),
                new SimpleElevator(() -> constElevator.MIN_HEIGHT, m_elevatorSubsystem));

    }

    @Override
    public boolean isFinished() {
        return stateFinished;
    }
}

package frc.robot.commands.ButtonBindings;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.Reset;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleAlgaeTimeout;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleElevatorTimeout;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class Algae1PreMove extends Command {
    private ElevatorSubsystem m_elevatorSubsystem;
    private CoralRollerSubsystem m_coralRollerSubsystem;
    private AlgaeArmSubsystem m_algaeArmSubsystem;

    private static boolean stateFinished;

    public Algae1PreMove(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_algaeArmSubsystem = algaeArmSubsystem;
        this.m_coralRollerSubsystem = rollerSubsystem;

        stateFinished = false;
    }

    @Override
    public void initialize() {
        if (m_elevatorSubsystem.isAtPosition2("A1P")) {
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
                new ParallelCommandGroup(
                        new SimpleElevatorTimeout(() -> constElevator.ALGAE1_PREMOVE, m_elevatorSubsystem),
                        new SimpleAlgaeTimeout(() -> constEndEffector.algaePivot.REMOVAL_ANGLE_BOTTOM,
                                m_algaeArmSubsystem)),
                new InstantCommand(() -> stateFinished = true));
    }

    private Command stage2() {
        return new SequentialCommandGroup(
                new SimpleRoller(() -> -constEndEffector.rollerSpeeds.DEFAULT_CORAL, m_coralRollerSubsystem),
                new ParallelDeadlineGroup(
                        new SimpleAlgaeTimeout(() -> 90.0, m_algaeArmSubsystem),
                        new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL_BOTTOM,
                                m_coralRollerSubsystem)),
                new SimpleAlgaeTimeout(() -> 0.0, 0.02, m_algaeArmSubsystem),
                new Reset(m_elevatorSubsystem, m_coralRollerSubsystem, m_algaeArmSubsystem),
                new InstantCommand(() -> stateFinished = true));
    }

    @Override
    public boolean isFinished() {
        return stateFinished;
    }
}

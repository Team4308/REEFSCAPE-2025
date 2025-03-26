package frc.robot.commands.CoralScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.Reset;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class FastL1 extends SequentialCommandGroup {
    public FastL1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                new Reset(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),
                new SimpleAlgae(() -> constEndEffector.algaePivot.REST_ANGLE, algaeArmSubsystem),
                new SimpleElevator(() -> constElevator.L1, elevatorSubsystem),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L1, rollerSubsystem),
                new SimpleElevator(() -> constElevator.L3, elevatorSubsystem),
                new SimpleElevator(() -> constElevator.MIN_HEIGHT, elevatorSubsystem));
    }
}

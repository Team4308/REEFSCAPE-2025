package frc.robot.commands.CoralScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.SimpleControl.SimpleAlgaeTimeout;
import frc.robot.commands.SimpleControl.SimpleElevatorTimeout;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class FastL2 extends SequentialCommandGroup {
    public FastL2(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                new SimpleAlgaeTimeout(() -> constEndEffector.algaePivot.REST_ANGLE,
                        algaeArmSubsystem),
                new SimpleElevatorTimeout(() -> constElevator.L2, elevatorSubsystem),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L23, rollerSubsystem),
                new SimpleElevatorTimeout(() -> constElevator.MIN_HEIGHT, elevatorSubsystem));
    }
}

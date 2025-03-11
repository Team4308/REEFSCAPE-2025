
package frc.robot.commands.CoralScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class FastL4 extends SequentialCommandGroup {
    public FastL4(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                new SimpleAlgae(() -> constEndEffector.algaePivot.MIN_ANGLE, algaeArmSubsystem),
                new SimpleElevator(() -> constElevator.MAX_HEIGHT, elevatorSubsystem),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L4, rollerSubsystem),
                new SimpleElevator(() -> constElevator.MIN_HEIGHT, elevatorSubsystem));
    }
}

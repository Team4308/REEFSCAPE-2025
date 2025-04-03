package frc.robot.commands.SimpleControl;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Constants.constElevator;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class SimpleAlgaeTimeout extends SequentialCommandGroup {
    public SimpleAlgaeTimeout(Supplier<Double> target, double timeout, AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                new SimpleAlgae(target, algaeArmSubsystem).withTimeout(Units.Seconds.of(timeout)));
    }

    public SimpleAlgaeTimeout(Supplier<Double> target, AlgaeArmSubsystem algaeArmSubsystem) {
        addCommands(
                new SimpleAlgae(target, algaeArmSubsystem).withTimeout(Units.Seconds.of(1)));
    }
}

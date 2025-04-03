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

public class SimpleElevatorTimeout extends SequentialCommandGroup {
    public SimpleElevatorTimeout(Supplier<Double> target, double timeout, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                new SimpleElevator(target, elevatorSubsystem).withTimeout(Units.Seconds.of(timeout)));
    }

    public SimpleElevatorTimeout(Supplier<Double> target, ElevatorSubsystem elevatorSubsystem) {
        addCommands(new SimpleElevator(target, elevatorSubsystem).withTimeout(Units.Seconds.of(1)));

    }
}

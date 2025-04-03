package frc.robot.commands.CoralScoring;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.Constants.constEndEffector;
import frc.robot.Ports.LED;
import frc.robot.Constants.constElevator;
import frc.robot.commands.Reset;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class FastL1 extends SequentialCommandGroup {
    public FastL1(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem, LEDSystem m_ledSubsystem) {
        addCommands(
                new Reset(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),
                new SimpleElevator(() -> constElevator.L1, elevatorSubsystem),
                new SimpleRoller(() -> constEndEffector.rollerSpeeds.L1, rollerSubsystem),
                new InstantCommand(() -> new XboxController(0).setRumble(RumbleType.kBothRumble, 1)),
                new InstantCommand(() -> m_ledSubsystem.setLedState("CoralShot")),
                new WaitCommand(1),
                new InstantCommand(() -> new XboxController(0).setRumble(RumbleType.kBothRumble, 0)),
                new InstantCommand(() -> m_ledSubsystem.clearStatus()));
    }
}

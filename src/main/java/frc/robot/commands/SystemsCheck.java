package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.constEndEffector;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlgaeRemoval.RemoveL1;
import frc.robot.commands.AlgaeRemoval.RemoveL2;
import frc.robot.commands.CoralScoring.FastL1;
import frc.robot.commands.CoralScoring.FastL2;
import frc.robot.commands.CoralScoring.FastL3;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

/*
run all swerve motors in one direction(translational movement)
run all swerve motors in a circle(rotational)

fastl3
fastl2
fastl1

removel1
removel2 

put coral in funnel
intake command
 */

public class SystemsCheck extends SequentialCommandGroup {
    public SystemsCheck(ElevatorSubsystem elevatorSubsystem, CoralRollerSubsystem rollerSubsystem,
            AlgaeArmSubsystem algaeArmSubsystem, SwerveSubsystem swerveSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(new WaitCommand(1), swerveSubsystem.driveCommand(() -> 5, () -> 0, () -> 0)),
                
                new ParallelDeadlineGroup(new WaitCommand(1), swerveSubsystem.driveCommand(() -> 0, () -> 5, () -> 0)),

                new ParallelDeadlineGroup(new WaitCommand(1), swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 5)),

                new WaitCommand(1),
                new InstantCommand(() -> swerveSubsystem.lock()),

                elevatorSubsystem.goToLevel(0),
                new FastL3(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),
                new FastL2(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),
                new FastL1(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),

                new RemoveL1(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),
                new RemoveL2(elevatorSubsystem, rollerSubsystem, algaeArmSubsystem),

                new DefaultRoller(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL, rollerSubsystem)
                        .until(() -> !rollerSubsystem.beamBreak.get()),
                new WaitCommand(1),
                new DefaultRoller(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL, rollerSubsystem)
                        .until(() -> rollerSubsystem.beamBreak.get()));
    }
}

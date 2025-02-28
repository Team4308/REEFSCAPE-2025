// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlgaePivotCommand;
import frc.robot.commands.CoralRollerCommand;
import frc.robot.subsystems.AlgaePivotSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;

public class RobotContainer {
  //subsystems
  private final AlgaePivotSubsystem m_algaePivotSubsystem;
  private final CoralRollerSubsystem m_coralRollerSubsystem;

  //commands
  private final AlgaePivotCommand algaePivotCommand;
  private final CoralRollerCommand coralRollerCommand;

  //controller wrapper
  public final XBoxWrapper controller1 = new XBoxWrapper(Constants.Mapping.controllerPort);


  public RobotContainer() {
    m_algaePivotSubsystem = new AlgaePivotSubsystem();
    m_coralRollerSubsystem = new CoralRollerSubsystem();
    algaePivotCommand = new AlgaePivotCommand(() -> null, m_algaePivotSubsystem);
    coralRollerCommand = new CoralRollerCommand(() -> null, m_coralRollerSubsystem);

    m_algaePivotSubsystem.setDefaultCommand(algaePivotCommand);
    m_coralRollerSubsystem.setDefaultCommand(coralRollerCommand);

    configureBindings();
  }

  private void configureBindings() {
    //While left trigger held, it will go to the fully up position (which should be about 144 degrees, guesstimate for knocking algae) and when it's not it should go back to resting
    controller1.LeftTrigger.whileTrue(new AlgaePivotCommand(() -> Constants.AlgaePivot.fullyUp, m_algaePivotSubsystem)).whileFalse(new AlgaePivotCommand(() -> Constants.AlgaePivot.resting, m_algaePivotSubsystem));
    controller1.A.whileTrue(new CoralRollerCommand(() -> Constants.CoralRoller.coralRollerSpeedL23, m_coralRollerSubsystem)); //A = l2 and l3 speed
    controller1.B.whileTrue(new CoralRollerCommand(() -> Constants.CoralRoller.coralRollerSpeedL4, m_coralRollerSubsystem)); //B = l4 speed
    controller1.X.whileTrue(new CoralRollerCommand(() -> 0.0, m_coralRollerSubsystem)); //stop movement rollers

    //test knocking off a piece of algae
    controller1.Y.whileTrue(new AlgaePivotCommand(() -> Constants.AlgaePivot.fullyUp, m_algaePivotSubsystem)).whileFalse(new AlgaePivotCommand(() -> Constants.AlgaePivot.resting, m_algaePivotSubsystem));
    controller1.Y.whileTrue(new CoralRollerCommand(() -> Constants.CoralRoller.coralRollerSpeedL4, m_coralRollerSubsystem)).whileFalse(new CoralRollerCommand(() -> 0.0, m_coralRollerSubsystem));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

/*

Constants have not been tuned or tested, so they have been assigned low variables for safety

   o
  /|\
  / \
 
 */
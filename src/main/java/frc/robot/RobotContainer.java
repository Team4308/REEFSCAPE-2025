// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RotateShooterSystem;

public class RobotContainer
{
  private final RotateShooterSystem m_rotateShooterSystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final ShooterCommand shooterCommand;
  private final RotateShooterCommand rotateShooterCommand;

  public RobotContainer()
  {
    m_rotateShooterSystem = new RotateShooterSystem();
    m_shooterSubsystem = new ShooterSubsystem();

    shooterCommand = new ShooterCommand(m_shooterSubsystem, () -> getShooterControl());
    m_shooterSubsystem.setDefaultCommand(shooterCommand);

    rotateShooterCommand = new RotateShooterCommand(m_rotateShooterSystem, () -> getRotateShooterControl());
    m_rotateShooterSystem.setDefaultCommand(rotateShooterCommand);

    configureBindings();
  }

  private void configureBindings()
  {
    stick1.X.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
    stick1.X.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));

    stick1.A.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> Constants.Shooter.shooterRPS));
    stick1.A.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> Constants.GamePieces.speaker.angle));

    stick1.Y.whileTrue(new ShooterCommand(m_shooterSubsystem, () -> 20.0));
    stick1.Y.whileTrue(new RotateShooterCommand(m_rotateShooterSystem, () -> m_rotateShooterSystem.autoAlignShooter()));
  }


  private double getShooterControl()
  {
    return 0.0;
  }

  private double getRotateShooterControl()
  {
    return 0.0;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}

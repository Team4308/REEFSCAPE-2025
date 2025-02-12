// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final LEDSystem m_ledSystem;
    private final ElevatorSubsystem m_elevator;

    public RobotContainer() {
        m_ledSystem = new LEDSystem();
        m_elevator = new ElevatorSubsystem(m_ledSystem);
        m_ledSystem.setElevator(m_elevator);

        CommandScheduler.getInstance().registerSubsystem(m_ledSystem);
        CommandScheduler.getInstance().registerSubsystem(m_elevator);

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public LEDSystem getLEDSystem() {
        return m_ledSystem;
    }

    public ElevatorSubsystem getElevator() {
        return m_elevator;
    }
}

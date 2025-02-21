// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final LEDSystem m_ledSystem;
    private final ElevatorSubsystem m_elevator;
    public final XBoxWrapper stick = new XBoxWrapper(Constants.Mapping.Controllers.kStick);
    public final XBoxWrapper stick1 = new XBoxWrapper(Constants.Mapping.Controllers.kStick1);
    public RobotContainer() {
        m_ledSystem = new LEDSystem();
        m_elevator = new ElevatorSubsystem();
        m_ledSystem.setElevator(m_elevator);

        CommandScheduler.getInstance().registerSubsystem(m_ledSystem);
        CommandScheduler.getInstance().registerSubsystem(m_elevator);

        configureBindings();
    }

    private void configureBindings() {
            stick.A.onTrue(new InstantCommand(() -> m_elevator.homeElevator()));
            stick.Up.onTrue(new InstantCommand(() -> m_elevator.goToLevel(1)));
            stick.Up.onFalse(new InstantCommand(() -> m_elevator.goToLevel(0)));
            stick.Right.onTrue(new InstantCommand(() -> m_elevator.goToLevel(2)));
            stick.Right.onFalse(new InstantCommand(() -> m_elevator.goToLevel(0)));

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

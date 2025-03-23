package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/*
 *                D
 *               1 2
 *              _____
 *           1 /     \ 2
 *        E 2 /       \ 1 C
 *          1 \       /2
 *         F 2 \_____/ 1 B
 *               1 2
 *                A
 *        (relative to side)
 *  Coral Shooting: (Side, Pole, Height)
 *  Algae Removal: (Side, Height)
 *  Intake (Side, Side^2)
 * 
 * 
 * 
 */

public class AutoChooser {
    public enum actions {

    }

    private static RobotContainer m_robotContainer;

    public boolean isUsingCustom = false;

    private final SendableChooser<Command> autoChooser;

    private double actionLength;
    private ArrayList<Command> commandList;

    public AutoChooser(RobotContainer robotContainer) {
        this.m_robotContainer = robotContainer;

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void perodic() {

    }
}
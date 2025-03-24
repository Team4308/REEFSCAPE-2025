package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.DropdownList;

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
 */

public class AutoChooser {
    private RobotContainer m_robotContainer;

    public boolean isUsingCustom = false;

    private final SendableChooser<Command> autoChooser;

    private int actionLength = 1;
    private ArrayList<Command> commandList;

    private ArrayList<String> actions;

    private HashMap<String, Pose2d> locationHashMap;
    private ArrayList<DropdownList<String>> steps;

    public AutoChooser(RobotContainer robotContainer) {
        this.m_robotContainer = robotContainer;

        setupMap();
        setupActions();
        makeSendableChooser();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("Using Custom?", false);
        SmartDashboard.putBoolean("+ Length", false);
        SmartDashboard.putBoolean("- Length", false);

        testing();
    }

    public Command getAutonomousCommand() {
        if (!isUsingCustom) {
            return autoChooser.getSelected();
        } else {
            return null;
        }
    }

    public void periodic() {
        dashboardHandler();
    }

    private void dashboardHandler() {
        isUsingCustom = SmartDashboard.getBoolean("Using Custom?", false);
        if (SmartDashboard.getBoolean("+ Length", false)) {
            actionLength++;
            actionLength = Math.min(actionLength, 15);
            SmartDashboard.putBoolean("+ Length", false);
            makeSendableChooser();
        } else if (SmartDashboard.getBoolean("- Length", false)) {
            actionLength--;
            actionLength = Math.max(actionLength, 1);
            SmartDashboard.putBoolean("- Length", false);
            makeSendableChooser();
        }
        SmartDashboard.putNumber("LENGTH", actionLength);

        getSendableChooser();

    }

    private void testing() {
        SmartDashboard.putData("Custom Dropdown", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("String Chooser");

                builder.addStringArrayProperty("Choice Array", () -> actions.toArray(new String[0]), null);
                builder.addStringProperty("active", null, null);
                builder.addStringProperty("active", null, null);
                builder.addIntegerProperty("Length", () -> actionLength, null);

            }
        });
    }

    private void makeSendableChooser() {
        steps = new ArrayList<DropdownList<String>>();
        for (int i = 1; i <= actionLength; i++) {
            steps.add(new DropdownList<String>());
            for (String action : actions) {
                steps.get(i - 1).addOption(action);
            }
            SmartDashboard.putData("Choice " + Integer.toString(i), steps.get(i - 1));
        }
        for (int i = actionLength + 1; i <= 15; i++) {
            SmartDashboard.putData("Choice " + Integer.toString(i), new DropdownList<String>());
        }
    }

    private void getSendableChooser() {
        var temp = new ArrayList<String>();
        for (int i = 1; i <= actionLength; i++) {

            temp.add(steps.get(i - 1).getSelected());
        }
        SmartDashboard.putStringArray("temparray", temp.toArray(new String[0]));
    }

    private void setupMap() {
        locationHashMap = new HashMap<String, Pose2d>();// very scuffed and def not working
        locationHashMap.put("A1", FieldLayout.REEF.A);
        locationHashMap.put("A2", FieldLayout.REEF.B);
        locationHashMap.put("B1", FieldLayout.REEF.C);
        locationHashMap.put("B2", FieldLayout.REEF.D);
        locationHashMap.put("C1", FieldLayout.REEF.E);
        locationHashMap.put("C2", FieldLayout.REEF.F);
        locationHashMap.put("D1", FieldLayout.REEF.G);
        locationHashMap.put("D2", FieldLayout.REEF.H);
        locationHashMap.put("E1", FieldLayout.REEF.I);
        locationHashMap.put("E2", FieldLayout.REEF.J);
        locationHashMap.put("F1", FieldLayout.REEF.K);
        locationHashMap.put("F2", FieldLayout.REEF.L);
    }

    private void setupActions() {
        actions = new ArrayList<String>();
        actions.add("action1");
        actions.add("action2");
        actions.add("action3");
        actions.add("action4");
        actions.add("action5");
    }
}
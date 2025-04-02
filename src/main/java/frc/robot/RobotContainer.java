// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driver;
import frc.robot.Constants.constEndEffector;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Reset;
import frc.robot.commands.SystemsCheck;
import frc.robot.commands.ButtonBindings.Algae1PreMove;
import frc.robot.commands.ButtonBindings.Algae2PreMove;
import frc.robot.commands.ButtonBindings.L2Algae1;
import frc.robot.commands.ButtonBindings.L2Algae2;
import frc.robot.commands.ButtonBindings.L2PreMove;
import frc.robot.commands.ButtonBindings.L3Algae1;
import frc.robot.commands.ButtonBindings.L3Algae2;
import frc.robot.commands.ButtonBindings.L3PreMove;
import frc.robot.commands.CoralScoring.FastL2;
import frc.robot.commands.CoralScoring.FastL3;
import frc.robot.commands.DefaultControl.DefaultAlgae;
import frc.robot.commands.DefaultControl.DefaultElevator;
import frc.robot.commands.DefaultControl.DefaultRoller;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
        // Controllers
        private final XBoxWrapper driver = new XBoxWrapper(Ports.Joysticks.DRIVER);
        private final XBoxWrapper operator = new XBoxWrapper(Ports.Joysticks.OPERATOR);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
        private final LEDSystem m_ledSubsystem;
        private final ElevatorSubsystem m_ElevatorSubsystem;
        private final AlgaeArmSubsystem m_AlgaeArmSubsystem;
        private final CoralRollerSubsystem m_CoralRollerSubsystem;

        // Commands
        private final DefaultRoller DefaultRollerCommand;
        private final DefaultAlgae DefaultAlgaeCommand;
        private final DefaultElevator DefaultElevatorCommand;

        private final L3Algae1 L3A1Command;

        private final SendableChooser<Command> autoChooser;

        private final Simulation m_simulation;

        private final Trigger coralIntakeTrigger;
        private final Trigger drivebaseAlignedTrigger;

        // Converts driver input into a field-relative ChassisSpeeds that is controlled
        // by angular velocity.
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver.getLeftY() * -1,
                        () -> driver.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driver.getRightX() * -1)
                        .deadband(Driver.DEADBAND)
                        .scaleTranslation(1.0)
                        .allianceRelativeControl(true);

        // Clone's the angular velocity input stream and converts it to a fieldRelative
        // input stream.
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
                        driver::getRightY)
                        .headingWhile(true);

        // Clone's the angular velocity input stream and converts it to a roboRelative
        // input stream.
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX())
                        .withControllerRotationAxis(() -> driver.getRightX())
                        .deadband(Driver.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driver.getLeftTrigger() * Math.PI) * (Math.PI * 2),
                                        () -> Math.cos(driver.getLeftTrigger() * Math.PI) * (Math.PI * 2))
                        .headingWhile(true);

        // Reef align
        SwerveInputStream driveToClosestLeftReef = driveDirectAngle.copy();
        SwerveInputStream driveToClosestRightReef = driveDirectAngle.copy();

        public RobotContainer() {
                m_ledSubsystem = new LEDSystem(RobotContainer.this);
                m_ElevatorSubsystem = new ElevatorSubsystem();
                m_ledSubsystem.setElevator(m_ElevatorSubsystem);
                m_AlgaeArmSubsystem = new AlgaeArmSubsystem();
                m_CoralRollerSubsystem = new CoralRollerSubsystem();
                m_simulation = new Simulation();
                m_simulation.setupsubsystems(m_ElevatorSubsystem, m_AlgaeArmSubsystem, m_CoralRollerSubsystem);

                DefaultRollerCommand = new DefaultRoller(() -> triggerRollerControl(), m_CoralRollerSubsystem);
                DefaultAlgaeCommand = new DefaultAlgae(() -> joystickAlgaeArm(), m_AlgaeArmSubsystem);
                DefaultElevatorCommand = new DefaultElevator(() -> joystickElevatorControl(), m_ElevatorSubsystem);

                L3A1Command = new L3Algae1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem);

                m_AlgaeArmSubsystem.setDefaultCommand(DefaultAlgaeCommand);
                m_CoralRollerSubsystem.setDefaultCommand(DefaultRollerCommand);
                m_ElevatorSubsystem.setDefaultCommand(DefaultElevatorCommand);

                CommandScheduler.getInstance().registerSubsystem(m_ledSubsystem);
                CommandScheduler.getInstance().registerSubsystem(m_ElevatorSubsystem);
                CommandScheduler.getInstance().registerSubsystem(m_AlgaeArmSubsystem);
                CommandScheduler.getInstance().registerSubsystem(m_CoralRollerSubsystem);

                coralIntakeTrigger = new Trigger(m_CoralRollerSubsystem::getBeamBreak);
                drivebaseAlignedTrigger = new Trigger(drivebase::isAligned);

                configureNamedCommands();
                configureDriverBindings();
                configureOperatorBindings();
                configureOtherTriggers();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureDriverBindings() {
                // Command driveFieldOrientedDirectAngle =
                // drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                // Command driveRobotOrientedAngularVelocity =
                // drivebase.driveFieldOriented(driveRobotOriented);
                // Command driveSetpointGen =
                // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
                // Command driveFieldOrientedDirectAngleKeyboard =
                // drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                // Command driveSetpointGenKeyboard = drivebase
                // .driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

                // driver.A.whileTrue(drivebase.updateClosestAlgaePose()
                // .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToAlgaeRemove)));
                driver.A.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToLeftReef)));
                driver.B.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToRightReef)));
                driver.X.whileTrue(drivebase.updateClosestStationPose()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToFarCoralStation)));
                driver.Y.whileTrue(drivebase.updateClosestStationPose()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToNearCoralStation)));
                driver.LB.onTrue((Commands.runOnce(drivebase::zeroGyro)));
                driver.RB.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }
        }

        private void configureOperatorBindings() {
                /*
                 * elevator failsafe(0,2,3)
                 * intaking
                 * auto:
                 * L2, L3, A1, A2
                 * L2A1, L2A2, L3A1, L3A2
                 * manual control roller, elevator, algae
                 */

                // Automatic Scoring
                operator.B.onTrue(new Reset(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                operator.A.onTrue(new FastL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                operator.X.onTrue(new L2PreMove(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                operator.Y.onTrue(new L3PreMove(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));

                // Automatic Algae Removal
                operator.LB
                                .onTrue(new Algae1PreMove(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                                                m_AlgaeArmSubsystem));
                operator.RB
                                .onTrue(new Algae2PreMove(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                                                m_AlgaeArmSubsystem));

                // Intake
                operator.Start.onTrue(new IntakeCommand(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL,
                                m_CoralRollerSubsystem));
                operator.Start.onTrue(new SimpleElevator(() -> 0.0, m_ElevatorSubsystem));

                // Dual Cycles
                operator.X.and(operator.LB)
                                .onTrue(new L2Algae1(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                                                m_AlgaeArmSubsystem));
                operator.X.and(operator.RB)
                                .onTrue(new L2Algae2(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                                                m_AlgaeArmSubsystem));
                operator.Y.and(operator.LB)
                                .onTrue(new L3Algae1(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                                                m_AlgaeArmSubsystem));
                operator.Y.and(operator.RB)
                                .onTrue(new L3Algae2(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                                                m_AlgaeArmSubsystem));

                /*
                 * L2A3
                 * shoot, spin to remove
                 * L3A3
                 * remove, while aligning to l3 then shoot
                 */

                // Elevator Failsafe
                operator.povUp.onTrue(m_ElevatorSubsystem.goToLevel(3));
                operator.povRight.onTrue(m_ElevatorSubsystem.goToLevel(0));
                operator.povDown.onTrue(m_ElevatorSubsystem.goToLevel(1));
                operator.povLeft.onTrue(m_ElevatorSubsystem.goToLevel(2));

                operator.LeftStickButton.whileTrue(drivebase.updateClosestStationPose()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToLeftReef)));
                operator.RightStickButton.onTrue(new SimpleAlgae(() -> 30.0, m_AlgaeArmSubsystem));
        }

        private void configureOtherTriggers() {
                coralIntakeTrigger.onTrue(new InstantCommand(() -> m_ledSubsystem.setLedState("Coral")));
                coralIntakeTrigger.onFalse(new InstantCommand(() -> {
                        if (m_ledSubsystem.getLedState().equals("Coral")) {
                                m_ledSubsystem.clearStatus();
                        }
                }));
                drivebaseAlignedTrigger.onTrue(new InstantCommand(() -> m_ledSubsystem.setLedState("Aligned")));
                drivebaseAlignedTrigger.onFalse(new InstantCommand(() -> {
                        m_ledSubsystem.clearTemporary();
                        ;

                }));
        }

        public void configureTeleopBindings() {
                coralIntakeTrigger.onTrue(new RunCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
                                .withTimeout(1.0).finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0)));
                // coralIntakeTrigger.onTrue(new RunCommand(() ->
                // operator.setRumble(RumbleType.kBothRumble, 1)).withTimeout(1.0).finallyDo(()
                // -> operator.setRumble(RumbleType.kBothRumble, 0)));

                drivebaseAlignedTrigger.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 1)));
                drivebaseAlignedTrigger
                                .onFalse(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));

                // drivebaseAlignedTrigger.onTrue(new InstantCommand(() ->
                // driver.setRumble(RumbleType.kBothRumble, 1)));
                // drivebaseAlignedTrigger.onFalse(new InstantCommand(() ->
                // driver.setRumble(RumbleType.kBothRumble, 0)));

                operator.X.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.A.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.Y.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.RB.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.LB.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("Intake Coral",
                                new IntakeCommand(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL,
                                                m_CoralRollerSubsystem));
                NamedCommands.registerCommand("L2 Preset",
                                new FastL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("L3 Preset",
                                new FastL3(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("Align To Left Reef",
                                drivebase.driveToPose(() -> drivebase.getClosestLeftReefPose())
                                                .until(drivebaseAlignedTrigger));
                NamedCommands.registerCommand("Align To Right Reef",
                                drivebase.driveToPose(() -> drivebase.getClosestRightReefPose())
                                                .until(drivebaseAlignedTrigger));
                NamedCommands.registerCommand("Align To Far Station",
                                drivebase.driveToPose(() -> drivebase.getClosestFarCoralStationPose())
                                                .until(drivebaseAlignedTrigger));
                NamedCommands.registerCommand("L2 Premove", m_ElevatorSubsystem.goToLevel(2));
                NamedCommands.registerCommand("L3A1 Stage1", L3A1Command.stage1());
                NamedCommands.registerCommand("L3A1 Stage2", L3A1Command.stage2());
                NamedCommands.registerCommand("Remove Low Algae and Shoot L2",
                                new L2Algae1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("Remove High Algae and Shoot L3",
                                new L3Algae2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("Remove High Algae and Shoot L2",
                                new L2Algae2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
        }

        public LEDSystem getLEDSystem() {
                return m_ledSubsystem;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void robotPeriodic() {
                Logger.recordOutput("/Subsystems/Algae/LoggedPose", new Pose3d(
                                new Translation3d(0.292, -0.2, m_ElevatorSubsystem.getPositionInMeters() + 0.3055),
                                new Rotation3d(0, Units.degreesToRadians(90)
                                                - Units.degreesToRadians(m_AlgaeArmSubsystem.getAlgaePosition()), 0)));
        }

        public void teleopInit() {
                configureTeleopBindings();
        }

        public void teleopPeriodic() {
        }

        public void simulationPerodic() {
                m_simulation.run();
        }

        public void disabledInit() {
                driver.setRumble(RumbleType.kBothRumble, 0);
                operator.setRumble(RumbleType.kBothRumble, 0);
                m_ledSubsystem.setLedState("Idle");
        }

        private double joystickAlgaeArm() {
                return -deadZone(operator.getLeftY()) * 7;
        }

        public double joystickElevatorControl() {
                return -deadZone(operator.getRightY()) / 10;
        }

        private double triggerRollerControl() {
                double isPos = deadZone(operator.getRightTrigger()) * 50;
                double isNeg = deadZone(operator.getLeftTrigger()) * 50;
                if (isPos > 0) {
                        return isPos;
                } else {
                        return -isNeg;
                }
        }

        public void runSystemsCheck() {
                new SystemsCheck(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem, drivebase)
                                .schedule();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        private static double deadZone(double integer) {
                if (Math.abs(integer) < 0.1) {
                        integer = 0;
                }
                return integer;
        }
}
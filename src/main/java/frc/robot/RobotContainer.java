// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Operator;
import frc.robot.Constants.constEndEffector;
import frc.robot.commands.AlgaeRemoval.RemoveL1;
import frc.robot.commands.AlgaeRemoval.RemoveL2;
import frc.robot.commands.CoralScoring.FastL1;
import frc.robot.commands.CoralScoring.FastL2;
import frc.robot.commands.CoralScoring.FastL3;
import frc.robot.commands.DefaultControl.DefaultAlgae;
import frc.robot.commands.DefaultControl.DefaultElevator;
import frc.robot.commands.DefaultControl.DefaultRoller;
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

        // Failsafe commands
        private final DefaultRoller ManualRollerCommand;
        private final DefaultAlgae ManualAlgaeCommand;
        private final DefaultElevator ManualElevatorCommand;

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
                        .deadband(Operator.DEADBAND)
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
                        .deadband(Operator.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driver.getLeftTrigger() *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driver.getLeftTrigger() *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
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

                ManualRollerCommand = new DefaultRoller(() -> triggerRollerControl(), m_CoralRollerSubsystem);
                ManualAlgaeCommand = new DefaultAlgae(() -> joystickAlgaeArm(), m_AlgaeArmSubsystem);
                ManualElevatorCommand = new DefaultElevator(() -> joystickElevatorControl(), m_ElevatorSubsystem);

                m_AlgaeArmSubsystem.setDefaultCommand(ManualAlgaeCommand);
                m_CoralRollerSubsystem.setDefaultCommand(ManualRollerCommand);
                m_ElevatorSubsystem.setDefaultCommand(ManualElevatorCommand);

                CommandScheduler.getInstance().registerSubsystem(m_ledSubsystem);
                CommandScheduler.getInstance().registerSubsystem(m_ElevatorSubsystem);
                CommandScheduler.getInstance().registerSubsystem(m_AlgaeArmSubsystem);
                CommandScheduler.getInstance().registerSubsystem(m_CoralRollerSubsystem);

                coralIntakeTrigger = new Trigger(() -> m_CoralRollerSubsystem.getBeamBreak());
                drivebaseAlignedTrigger = new Trigger(() -> drivebase.isAligned());

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

                driver.LB.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToLeftReef)));
                driver.RB.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToRightReef)));

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }
                if (Robot.isSimulation()) {
                        driver.Y.onTrue(Commands
                                        .runOnce(() -> drivebase.resetOdometry(new Pose2d(15, 4, new Rotation2d()))));
                        driver.A.whileTrue(drivebase.sysIdDriveMotorCommand());
                }
                if (DriverStation.isTest()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                                         // above!

                        driver.X.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        driver.Y.whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
                        driver.Start.onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        driver.Back.whileTrue(drivebase.centerModulesCommand());
                        driver.RightStickButton.onTrue(Commands.none());
                } else {
                        driver.A.onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        driver.X.onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                        driver.Y.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        driver.B.whileTrue(
                                        drivebase.driveToPose(
                                                        new Pose2d(new Translation2d(4, 4),
                                                                        Rotation2d.fromDegrees(0))));
                        driver.Start.whileTrue(Commands.none());
                        driver.Back.whileTrue(Commands.none());
                }
        }

        private void configureOperatorBindings() {
                // Automatic Scoring
                operator.X.onTrue(new FastL1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                operator.B.onTrue(new FastL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                operator.Y.onTrue(new FastL3(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                // operator.Y.onTrue(new FastL4(m_ElevatorSubsystem, m_CoralRollerSubsystem,
                // m_AlgaeArmSubsystem));

                // Automatic Algae Removal
                operator.LeftStickButton
                                .onTrue(new RemoveL1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                operator.RightStickButton
                                .onTrue(new RemoveL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));

                // *** These are failsafes, that should be already covered by the previous
                // commands ***
                // Coral
                operator.Start.onTrue(new DefaultRoller(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL,
                                m_CoralRollerSubsystem)
                                .until(() -> !m_CoralRollerSubsystem.beamBreak.get())); // INTAKING
                operator.Back.onTrue(new DefaultRoller(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL,
                                m_CoralRollerSubsystem)
                                .until(() -> m_CoralRollerSubsystem.beamBreak.get())); // SHOOTING

                // Algae
                operator.RB.onTrue(new InstantCommand(() -> m_AlgaeArmSubsystem
                                .setAlgaePosition(Constants.constEndEffector.algaePivot.REMOVAL_ANGLE)))
                                .onFalse((new InstantCommand(() -> m_AlgaeArmSubsystem
                                                .setAlgaePosition(Constants.constEndEffector.algaePivot.MAX_ANGLE)))); // Set
                                                                                                                       // position
                                                                                                                       // to
                                                                                                                       // remove
                                                                                                                       // algae
                operator.RB.onTrue(new DefaultRoller(() -> constEndEffector.rollerSpeeds.ALGAE_REMOVAL,
                                m_CoralRollerSubsystem))
                                .onFalse(new DefaultRoller(() -> 0.0, m_CoralRollerSubsystem));

                // Elevator
                operator.povUp.onTrue(m_ElevatorSubsystem.goToLevel(3));
                operator.povRight.onTrue(m_ElevatorSubsystem.goToLevel(2));
                operator.povDown.onTrue(m_ElevatorSubsystem.goToLevel(0));
                operator.povLeft.onTrue(m_ElevatorSubsystem.goToLevel(1));
                operator.A.onTrue(m_ElevatorSubsystem.goToLevel(0));
        }

        private void configureOtherTriggers() {
                drivebaseAlignedTrigger.onTrue(new InstantCommand(() -> m_ledSubsystem.setLedState("Aligned")));
                drivebaseAlignedTrigger.onFalse(new InstantCommand(() -> {
                        if (m_ledSubsystem.getLedState().equals("Aligned")) {
                                m_ledSubsystem.setLedState(m_ledSubsystem.previousState);
                        }
                }));

                coralIntakeTrigger.onTrue(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
                                .withTimeout(1).finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0)));
                drivebaseAlignedTrigger.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 1))
                                .withTimeout(1).finallyDo(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("Intake Coral",
                                new DefaultRoller(() -> constEndEffector.rollerSpeeds.DEFAULT_CORAL,
                                                m_CoralRollerSubsystem)
                                                .until(() -> !m_CoralRollerSubsystem.beamBreak.get()));
                NamedCommands.registerCommand("L2 Preset",
                                new FastL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("L3 Preset",
                                new FastL3(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("Remove Algae L1",
                                new RemoveL1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
                NamedCommands.registerCommand("Remove Algae L2",
                                new RemoveL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
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

        public void periodic() {
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
                return -deadZone(operator.getLeftY()) * 5;
        }

        private double joystickElevatorControl() {
                return -deadZone(operator.getRightY()) / 20;
        }

        private double triggerRollerControl() {
                double isPos = deadZone(operator.getRightTrigger()) * 15;
                double isNeg = deadZone(operator.getLeftTrigger()) * 15;
                if (isPos > 0) {
                        return isPos;
                } else {
                        return -isNeg;
                }
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        private static double deadZone(double integer) {
                if (Math.abs(integer) < Operator.DEADBAND) {
                        integer = 0;
                }
                return integer;
        }
}
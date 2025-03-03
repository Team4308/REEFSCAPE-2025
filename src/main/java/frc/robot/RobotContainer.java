// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.time.Instant;

import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.SwerveInputStream;

import frc.robot.Constants.Operator;
import frc.robot.commands.Intake;
import frc.robot.commands.AlgaeRemoval.RemoveL1;
import frc.robot.commands.AlgaeRemoval.RemoveL2;
import frc.robot.commands.CoralScoring.FastL1;
import frc.robot.commands.CoralScoring.FastL2;
import frc.robot.commands.CoralScoring.FastL3;
import frc.robot.commands.CoralScoring.FastL4;
import frc.robot.commands.SimpleControl.SimpleAlgae;
import frc.robot.commands.SimpleControl.SimpleElevator;
import frc.robot.commands.SimpleControl.SimpleRoller;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSystem;

public class RobotContainer {

  // Controllers
  private final XBoxWrapper driver = new XBoxWrapper(Ports.Joysticks.DRIVER);
  private final XBoxWrapper operator = new XBoxWrapper(Ports.Joysticks.OPERATOR);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private final LEDSystem m_ledSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final AlgaeArmSubsystem m_AlgaeArmSubsystem;
  private final CoralRollerSubsystem m_CoralRollerSubsystem;

  // for testing purposes
  private final SimpleAlgae manualAlgaeCommand;
  private final SimpleRoller manualRollerCommand;
  private final SimpleElevator manualElevatorCommand;

  // Converts driver input into a field-relative ChassisSpeeds that is controlled
  // by angular velocity.
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .withControllerRotationAxis(driver::getRightX)
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
      .withControllerRotationAxis(() -> driver.getLeftTrigger())
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

  public RobotContainer() {
    m_ledSubsystem = new LEDSystem();
    m_ElevatorSubsystem = new ElevatorSubsystem();
    m_ledSubsystem.setElevator(m_ElevatorSubsystem);
    m_AlgaeArmSubsystem = new AlgaeArmSubsystem();
    m_CoralRollerSubsystem = new CoralRollerSubsystem();

    manualAlgaeCommand = new SimpleAlgae(() -> null, m_AlgaeArmSubsystem);
    manualRollerCommand = new SimpleRoller(() -> null, m_CoralRollerSubsystem);
    manualElevatorCommand = new SimpleElevator(null, m_ElevatorSubsystem);

    m_AlgaeArmSubsystem.setDefaultCommand(manualAlgaeCommand);
    m_CoralRollerSubsystem.setDefaultCommand(manualRollerCommand);
    m_ElevatorSubsystem.setDefaultCommand(manualElevatorCommand);

    CommandScheduler.getInstance().registerSubsystem(m_ledSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_ElevatorSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_AlgaeArmSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_CoralRollerSubsystem);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  private void configureBindings() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  public void configureDriverBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driver.Start.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driver.A.whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driver.X.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driver.Y.whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driver.Start.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driver.Back.whileTrue(drivebase.centerModulesCommand());
      driver.LB.onTrue(Commands.none());
      driver.RightStickButton.onTrue(Commands.none());
    } else {
      driver.A.onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driver.X.onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driver.B.whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driver.Start.whileTrue(Commands.none());
      driver.Back.whileTrue(Commands.none());
      driver.LB.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driver.RB.onTrue(Commands.none());
    }

  }

  public void configureOperatorBindings() {
    // Coral
    operator.Start.onTrue(new Intake(m_CoralRollerSubsystem));
    operator.Back.whileTrue(new InstantCommand(() -> m_CoralRollerSubsystem.setRollerOutput(15)))
        .onFalse(new InstantCommand(() -> m_CoralRollerSubsystem.stopControllers()));

    // Algae
    operator.RB.onTrue(new InstantCommand(
        () -> m_AlgaeArmSubsystem.setAlgaePosition(Constants.EndEffector.algaePositions.removeAlgaePosition)));
    operator.RB.onTrue(new InstantCommand(() -> m_CoralRollerSubsystem.setRollerOutput(-15)));
    operator.RB.onFalse((new InstantCommand(
        () -> m_AlgaeArmSubsystem.setAlgaePosition(Constants.EndEffector.algaePositions.minPosition))));
    operator.RB.onFalse(new InstantCommand(() -> m_CoralRollerSubsystem.stopControllers()));

    // Elevator
    operator.povUp.onTrue(new InstantCommand(() -> m_ElevatorSubsystem.goToLevel(1)));
    operator.povRight.onTrue(new InstantCommand(() -> m_ElevatorSubsystem.goToLevel(2)));
    operator.povDown.onTrue(new InstantCommand(() -> m_ElevatorSubsystem.goToLevel(3)));
    operator.povLeft.onTrue(new InstantCommand(() -> m_ElevatorSubsystem.goToLevel(4)));
    operator.LB.onTrue(new InstantCommand(() -> m_ElevatorSubsystem.goToLevel(0)));

    // Automatic Scoring
    operator.A.onTrue(new FastL1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
    operator.B.onTrue(new FastL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
    operator.X.onTrue(new FastL3(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
    operator.Y.onTrue(new FastL4(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));

    // Automatic Algae Removal
    operator.LeftStickButton.onTrue(new RemoveL1(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
    operator.RightStickButton.onTrue(new RemoveL2(m_ElevatorSubsystem, m_CoralRollerSubsystem, m_AlgaeArmSubsystem));
  }

  public LEDSystem getLEDSystem() {
    return m_ledSubsystem;
  }

  public ElevatorSubsystem getElevator() {
    return m_ElevatorSubsystem;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void periodic() {
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public static double deadZone(double integer) {
    if (0.06 >= integer && integer >= -0.06) {
      integer = 0;
    }
    return integer;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  // Controllers
  private final XBoxWrapper driver = new XBoxWrapper(Constants.Mapping.Controllers.driver);
  private final XBoxWrapper operator = new XBoxWrapper(Constants.Mapping.Controllers.operator);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .withControllerRotationAxis(driver::getRightX)
      .deadband(Operator.DEADBAND)
      .scaleTranslation(1.0)
      .allianceRelativeControl(true);
      
  // Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
      driver::getRightY)
      .headingWhile(true);

   // Clone's the angular velocity input stream and converts it to a roboRelative input stream.
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

  ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  EndEffectorSubsystem m_EndEffectorSubsystem = new EndEffectorSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  private void configureBindings() {
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
    shooterControl();
    elevatorControl();
    algaeControl();
  }

  public void algaeControl() {
    double algaeJoystick = deadzone(operator.getRightY());
    algaeJoystick = DoubleUtils.mapRangeNew(algaeJoystick, -1, 1, -50, 50);
    m_EndEffectorSubsystem.setArmOutput(algaeJoystick);
  }

  public void shooterControl() {
    double rollerJoystickPos = deadzone(operator.getRightTrigger());
    double rollerJoystickNeg = deadzone(operator.getLeftTrigger());
    double rollerJoystick = 0;
    if (rollerJoystickPos >= 0) {
      rollerJoystick = rollerJoystickPos;
    } else if (rollerJoystickNeg >= 0) {
      rollerJoystick = -rollerJoystickNeg;
    }
    rollerJoystick = DoubleUtils.mapRangeNew(rollerJoystick, -1, 1, -50, 50);
    m_EndEffectorSubsystem.setArmOutput(rollerJoystick);
  }

  public void elevatorControl() {
    double elevatorJoystick = deadzone(operator.getLeftY());
    elevatorJoystick = DoubleUtils.mapRangeNew(elevatorJoystick, -1, 1, -50, 50);
    if (elevatorJoystick == 0) {
      elevatorJoystick = 1;
    }
    m_ElevatorSubsystem.setMotorSpeed(elevatorJoystick);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public static double deadzone(double integer) {
    if (0.06 >= integer && integer >= -0.06) {
      integer = 0;
    }
    return integer;
  }
}

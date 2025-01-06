package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.ctre.phoenix.schedulers.SequentialScheduler;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

  // Subsystems
  private final SwerveSubsystem drivebase;

  // Commands

  // Controllers
  private final XBoxWrapper driver = new XBoxWrapper(Constants.Mapping.Controllers.driver);
  private final XBoxWrapper operator = new XBoxWrapper(Constants.Mapping.Controllers.operator);

  // Auto
  private final SendableChooser<Command> autonomousChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Subsystem Instantiations
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));

    configureNamedCommands();
    
    // Command Instantiations

    autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autonomousChooser);

    // Configure the trigger bindings
    configureBindings();

    Command driveAngularVelocity = drivebase.driveVelocity(
        () -> MathUtil.applyDeadband(driver.getLeftY(), Controller.Driver.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), Controller.Driver.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getRightX(), Controller.Driver.RIGHT_X_DEADBAND));
    
    Command drivePresetAdvanced = drivebase.driveVelocityAdv(
        () -> MathUtil.applyDeadband(driver.getLeftX(), Controller.Driver.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftY(), Controller.Driver.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getRightX(), Controller.Driver.RIGHT_X_DEADBAND),
              driver.getYButtonPressed(), driver.getAButtonPressed(), 
              driver.getXButtonPressed(), driver.getBButtonPressed());

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveAngularVelocity : drivePresetAdvanced);
  }

  private void configureBindings() {
    driver.Y.onTrue(Commands.runOnce(drivebase::zeroGyro));

    driver.Down.onTrue(Commands.runOnce(drivebase::setupPathPlanner));
    // TESTING ONLY FOR TUNING PATHPLANNER PIDS, DISABLE WHEN NOT TUNING

    driver.A.whileTrue(drivebase.aimAtSpeaker(2));
    driver.B.whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                            new Pose2d(new Translation2d(2, 5.5), Rotation2d.fromDegrees(0)))
                        ));
  }

  public void configureNamedCommands() {
  } 

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public void zeroGyroOnTeleop() {
    drivebase.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void stopRumble() {
    driver.setLeftRumble(0.0);
    driver.setRightRumble(0.0);
    operator.setLeftRumble(0.0);
    operator.setRightRumble(0.0);
  }

  public void disabledActions() {
  }

  // Gets rid of the yellow errors in Robot.java
  public void ewyellowerrors() {
  }
}
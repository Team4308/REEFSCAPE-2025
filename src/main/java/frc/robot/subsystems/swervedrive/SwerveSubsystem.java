// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import ca.team4308.absolutelib.wrapper.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.FieldLayout;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private static final LoggedTunableNumber kAngleP = new LoggedTunableNumber("Swerve/Auton/Angle/kP",
      Constants.Swerve.Auton.Angle.kP);
  private static final LoggedTunableNumber kAngleI = new LoggedTunableNumber("Swerve/Auton/Angle/kI",
      Constants.Swerve.Auton.Angle.kI);
  private static final LoggedTunableNumber kAngleD = new LoggedTunableNumber("Swerve/Auton/Angle/kD",
      Constants.Swerve.Auton.Angle.kD);

  private static final LoggedTunableNumber kTranslationP = new LoggedTunableNumber("Swerve/Auton/Drive/kP",
      Constants.Swerve.Auton.Translation.kP);
  private static final LoggedTunableNumber kTranslationI = new LoggedTunableNumber("Swerve/Auton/Drive/kI",
      Constants.Swerve.Auton.Translation.kI);
  private static final LoggedTunableNumber kTranslationD = new LoggedTunableNumber("Swerve/Auton/Drive/kD",
      Constants.Swerve.Auton.Translation.kD);

  private PIDConstants ANGLE_CONTROLLER;
  private PIDConstants TRANSLATION_CONTROLLER;

  private PPHolonomicDriveController ALIGN_CONTROLLER;

  private final SwerveDrive swerveDrive;

  private final boolean visionDriveTest = true;
  private Vision vision;

  private Pose2d targetPose = new Pose2d();

  public Pose2d nearestPoseToLeftReef = new Pose2d();
  public Pose2d nearestPoseToRightReef = new Pose2d();
  public Pose2d nearestPoseToAlgaeRemove = new Pose2d();
  public Pose2d nearestPoseToFarCoralStation = new Pose2d();
  public Pose2d nearestPoseToNearCoralStation = new Pose2d();

  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Robot Pose", Pose2d.struct).publish();

  public SwerveSubsystem(File directory) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Only set to true if controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // Correct for skew that gets worse as angular velocity increases. Start
                                             // with a coefficient of 0.1. Could be negative
    swerveDrive.setAngularVelocityCompensation(false, false, 0.1); // Resync absolute encoders and motor encoders
                                                                   // periodically when they are not moving.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    if (visionDriveTest) {
      setupPhotonVision();
      // Stop the odometry thread while running vision to synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    ANGLE_CONTROLLER = new PIDConstants(kAngleP.get(), kAngleI.get(), kAngleD.get());
    TRANSLATION_CONTROLLER = new PIDConstants(kTranslationP.get(), kTranslationI.get(), kTranslationD.get());

    ALIGN_CONTROLLER = new PPHolonomicDriveController( // PPHolonomicController is the built in path following
                                                       // controller for
                                                       // holonomic drive trains
        // Translation PID constants
        new PIDConstants(4.0, 0.0, 0.0),
        // Rotation PID constants
        new PIDConstants(2.0, 0.0, 0.0));

    setupPathPlanner();
    // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyro));
    // Don't use this unless needed, orientation is correct right now
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg,
        controllerCfg,
        Constants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
            Rotation2d.fromDegrees(0)));
  }

  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic() {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
    checkTunableValues();

    publisher.set(getPose());

    // SmartDashboard.putBoolean("Aligned?", isAligned());
    Logger.recordOutput("Swerve/Is Aligned?", isAligned());
    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/Velocity", getRobotVelocity());
  }

  @Override
  public void simulationPeriodic() {
  }

  public void checkTunableValues() {
    if (!Constants.LoggedDashboard.TUNING_MODE) {
      return;
    }
    // Only update LoggedTunableNumbers when enabled
    if (DriverStation.isEnabled()) {
      LoggedTunableNumber.ifChanged(
          hashCode(), () -> ANGLE_CONTROLLER = new PIDConstants(kAngleP.get(), kAngleI.get(), kAngleD.get()),
          kAngleP, kAngleI, kAngleD);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> TRANSLATION_CONTROLLER = new PIDConstants(kTranslationP.get(), kTranslationI.get(),
              kTranslationD.get()),
          kTranslationP, kTranslationI, kTranslationD);
    }
  }

  // Setup AutoBuilder for PathPlanner.
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably store this in
    // your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry, // called if auto has starting pose
          this::getRobotVelocity, // must be robot relative
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
                                                       // controller for holonomic drive trains
            // Translation PID constants
            TRANSLATION_CONTROLLER,
            // Rotation PID constants
            ANGLE_CONTROLLER),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Preload PathPlanner Path finding. IF USING CUSTOM PATHFINDER ADD BEFORE THIS
    // LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public Pose2d getClosestLeftReefPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.REEF.RED_LEFT_REEF_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.REEF.BLUE_LEFT_REEF_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestRightReefPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.REEF.RED_RIGHT_REEF_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.REEF.BLUE_RIGHT_REEF_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestAlgaeRemovePose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.ALGAE.RED_ALGAE_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.ALGAE.BLUE_ALGAE_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestFarCoralStationPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.RED_FAR_STATION_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.BLUE_FAR_STATION_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestNearCoralStationPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.RED_NEAR_STATION_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.BLUE_NEAR_STATION_POSES);
    }
    return nearestPose;
  }

  public Command updateClosestReefPoses() {
    return this.runOnce(() -> {
      nearestPoseToLeftReef = getClosestLeftReefPose();
      nearestPoseToRightReef = getClosestRightReefPose();
    });
  }

  public Command updateClosestAlgaePose() {
    return this.runOnce(() -> {
      nearestPoseToAlgaeRemove = getClosestAlgaeRemovePose();
    });
  }

  public Command updateClosestStationPose() {
    return this.runOnce(() -> {
      nearestPoseToFarCoralStation = getClosestFarCoralStationPose();
      nearestPoseToNearCoralStation = getClosestNearCoralStationPose();
    });
  }

  public boolean isTranslationAligned() {
    Translation2d currentTranslation2d = getPose().getTranslation();
    Translation2d targetTranslation2d = targetPose.getTranslation();
    if (currentTranslation2d.getDistance(targetTranslation2d) < Swerve.Align.Translation.TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isHeadingAligned() {
    double currentHeading = getPose().getRotation().getDegrees();
    double targetHeading = targetPose.getRotation().getDegrees();
    if ((Math.abs(currentHeading - targetHeading) < Swerve.Align.Heading.TOLERANCE)
        || (Math.abs(currentHeading + targetHeading) < Swerve.Align.Heading.TOLERANCE)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAligned() {
    if (isTranslationAligned() && isHeadingAligned()) {
      return true;
    } else {
      return false;
    }
  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    return defer(() -> driveToPose(pose.get()));
  }

  public Command driveToPose(Pose2d pose) {
    // Change target pose
    targetPose = pose;

    // Create the constraints to use while pathfinding
    @SuppressWarnings("unused")
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 3.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Create the goal state
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = pose;

    // // // Since AutoBuilder is configured, we can use it to build pathfinding
    // // commands
    // Pose2d tValues = targetPose.relativeTo(getPose());
    // double pythagoreanDistance = Math.sqrt(Math.pow(tValues.getX(), 2) + Math.pow(tValues.getY(), 2));
    // if (Math.abs(pythagoreanDistance) > 1) {
    //   return AutoBuilder.pathfindToPose(
    //       pose,
    //       constraints,
    //       edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    //   ).andThen(run(() -> swerveDrive.drive(DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(getPose(), goalState))));

    // }

    // PID only test
    return run(() -> swerveDrive.drive(ALIGN_CONTROLLER.calculateRobotRelativeSpeeds(getPose(),
        goalState)));
  }

  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  public Command aimAtTarget(Cameras camera) {
    return run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent()) {
        var result = resultO.get();
        if (result.hasTargets()) {
          drive(getTargetSpeeds(0,
              0,
              Rotation2d.fromDegrees(result.getBestTarget()
                  .getYaw()))); // Not sure if this will work, more math may be required.
        }
      }
    });
  }

  // Swerve drive with Setpoint Generator from 254, implemented by PathPlanner
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
        swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
            swerveDrive.getStates(),
            DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  // Field Relative Drive with Setpoint Generator
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

  // SysID Drive Motors Characterization
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  // SysID Angle Motors Characterization
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  // Command to drive the robot using translative values and heading as angular
  // velocity.
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  // Command to drive the robot using translative values and heading as a
  // setpoint.
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  // Primary method for controlling the drivebase, for which the swerve drive
  // methods base on.
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled since it shouldn't be used
                                                                    // most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  // Resets the gyro angle to zero and resets odometry to the same position, but
  // facing toward 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // Checks if alliance is red, false if blue or not available.
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  // If red alliance rotate the robot 180 after the drivebase zero command
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  // Gets the current field-relative velocity (x, y and omega) of the robot
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  // Gets the current velocity (x, y and omega) of the robot
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Lock the swerve drive to prevent it from moving.
  public void lock() {
    swerveDrive.lockPose();
  }

  // Gets the current pitch angle of the robot, as reported by the imu.
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

}

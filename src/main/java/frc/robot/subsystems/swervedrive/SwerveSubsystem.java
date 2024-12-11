// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import ca.team4308.absolutelib.wrapper.LoggedTunableNumber;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{

  private Vision vision;
  private final SwerveDrive swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final boolean visionDriveTest = false;
  private boolean resetHeading = false;

  private LoggedTunableNumber angularVelocityCoeff = new LoggedTunableNumber(
      "Swerve/Teleop/AngularVelocityCoeff", Constants.Swerve.angularVelocityCoeff);

  private static final LoggedTunableNumber kAngleP = new LoggedTunableNumber("Swerve/Auton/kAngleP",
      Constants.Swerve.Auton.AngleControl.kP);
  private static final LoggedTunableNumber kAngleI = new LoggedTunableNumber("Swerve/Auton/kAngleI",
      Constants.Swerve.Auton.AngleControl.kI);
  private static final LoggedTunableNumber kAngleD = new LoggedTunableNumber("Swerve/Auton/kAngleD",
      Constants.Swerve.Auton.AngleControl.kD);

  private static final LoggedTunableNumber kTranslationP = new LoggedTunableNumber("Swerve/Auton/kTranslationP",
      Constants.Swerve.Auton.TranslationControl.kP);
  private static final LoggedTunableNumber kTranslationI = new LoggedTunableNumber("Swerve/Auton/kTranslationI",
      Constants.Swerve.Auton.TranslationControl.kI);
  private static final LoggedTunableNumber kTranslationD = new LoggedTunableNumber("Swerve/Auton/kTranslationD",
      Constants.Swerve.Auton.TranslationControl.kD);

  private PIDConstants ANGLE_CONTROLLER;
  private PIDConstants TRANSLATION_CONTROLLER;

  public SwerveSubsystem(File directory) {
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Swerve.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); 
    // Heading correction should only be used while controlling the robot via,angle.
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(false,false,0.1); 
    // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,1); 
    // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    swerveDrive.pushOffsetsToEncoders(); 
    // Set the absolute encoder to be used over the internal encoder and push the offsets onto it.

    if (visionDriveTest) {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    ANGLE_CONTROLLER = new PIDConstants(kAngleP.get(), kAngleI.get(), kAngleD.get());
    TRANSLATION_CONTROLLER = new PIDConstants(kTranslationP.get(), kTranslationI.get(), kTranslationD.get());

    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.Swerve.MAX_SPEED);
  }

  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public void checkTunableValues() {
        if (!Constants.LoggedDashboard.tuningMode) {
            return;
        }
        if (DriverStation.isEnabled()) {
          LoggedTunableNumber.ifChanged(
              hashCode(), () -> ANGLE_CONTROLLER = new PIDConstants(kAngleP.get(), kAngleI.get(), kAngleD.get()),
                                                                    kAngleP, kAngleI, kAngleD);
          LoggedTunableNumber.ifChanged(
              hashCode(),
              () -> TRANSLATION_CONTROLLER = new PIDConstants(kTranslationP.get(), kTranslationI.get(), kTranslationD.get()), 
                                                              kTranslationP, kTranslationI, kTranslationD);
          LoggedTunableNumber.ifChanged(hashCode(), () -> swerveDrive.setAngularVelocityCompensation(true,
                              true,angularVelocityCoeff.get()), angularVelocityCoeff);
        }
    }

  @Override
  public void periodic() {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }

    checkTunableValues();
  }

  @Override
  public void simulationPeriodic() {
  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            TRANSLATION_CONTROLLER,
            // Translation PID constants
            ANGLE_CONTROLLER,
            // Rotation PID constants
            4.5,
            // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public double getDistanceToSpeaker() {
    Pose3d speakerCenterPose = new Pose3d();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerCenterPose = Constants.GamePieces.Speaker.kSpeakerCenterBlue;
    } else {
      speakerCenterPose = Constants.GamePieces.Speaker.kSpeakerCenterRed;
    }
    // int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    // Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
    return getPose().getTranslation().getDistance(speakerCenterPose.toPose2d().getTranslation());
  }

  public Rotation2d getSpeakerYaw() {
    int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
    Translation2d relativeTrl = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }


  // Aim robot at speaker
  public Command aimAtSpeaker(double tolerance) {
    SwerveController controller = swerveDrive.getSwerveController();
    return run(
        () -> {
          drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,
              0,
              controller.headingCalculate(getHeading().getRadians(),
                  getSpeakerYaw().getRadians()),
              getHeading()));
        }).until(() -> Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) < tolerance);
  }

  // Aim robot at target of Photonvision camera
  public Command aimAtTarget(PhotonCamera camera) {

    return run(() -> {
      PhotonPipelineResult result = camera.getLatestResult();
      if (result.hasTargets()) {
        drive(getTargetSpeeds(0,
            0,
            Rotation2d.fromDegrees(result.getBestTarget()
                .getYaw()))); // Not sure if this will work, more math may be required.
      }
    });
  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  public Command driveVelocityAdv(DoubleSupplier translationX, DoubleSupplier translationY, 
                                    DoubleSupplier angularRotationX, BooleanSupplier lookAway, 
                                    BooleanSupplier lookTowards, BooleanSupplier lookLeft, BooleanSupplier lookRight) {
    return run(() -> {
      double headingX = 0;
      double headingY = 0;

      // Face Away from Drivers
      if (lookAway.getAsBoolean()) {
        headingY = -1;
      }
      // Face Right
      if (lookRight.getAsBoolean()) {
        headingX = 1;
      }
      // Face Left
      if (lookLeft.getAsBoolean()) {
        headingX = -1;
      }
      // Face Towards the Drivers
      if (lookTowards.getAsBoolean()) {
        headingY = 1;
      }

      // Prevent Movement After Auto
      if (resetHeading == true) {
        if (headingX == 0 && headingY == 0 && Math.abs(angularRotationX.getAsDouble()) == 0) {
          // Get the curret Heading
          Rotation2d currentHeading = getHeading();

          // Set the Current Heading to the desired Heading
          headingX = currentHeading.getSin();
          headingY = currentHeading.getCos();
        }
        // Dont reset Heading Again
        resetHeading = false;
      }

      ChassisSpeeds desiredSpeeds = getTargetSpeeds(translationX.getAsDouble(), translationY.getAsDouble(),
                                                    headingX, headingY);
      Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

      if (headingX == 0 && headingY == 0 && Math.abs(angularRotationX.getAsDouble()) > 0) {
        resetHeading = true;
        swerveDrive.drive(SwerveMath.scaleTranslation(translation, 1.0), 
                        angularRotationX.getAsDouble() * 
                        swerveDrive.getMaximumAngularVelocity() * 1.0,
                        true, false);
      } else {
        swerveDrive.drive(SwerveMath.scaleTranslation(translation, 1.0),
                          desiredSpeeds.omegaRadiansPerSecond,
                          true,false);
      }
    });
  }

  // Angular Velocity Drive Mode
  public Command driveVelocity(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      double transX = translationX.getAsDouble();
      double transY = translationY.getAsDouble();
      double rotationX = angularRotationX.getAsDouble();
      if (DriverStation.getAlliance().get() == Alliance.Blue ) {
        transX *= -1.0;
        transY *= -1.0;
        rotationX *= -1.0;
      }
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            transX * swerveDrive.getMaximumVelocity(),
                            transY * swerveDrive.getMaximumVelocity()), 1.0),
                        Math.pow(rotationX, 3) * swerveDrive.getMaximumAngularVelocity()
                         * 1.0, true,false);
    });
  }
  // Direct Angle Drive Mode
  public Command driveDirectAngle(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
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
          swerveDrive.getMaximumVelocity()));
    });
  }

  // Direct Angle Drive Mode (Simulation)
  public Command simDriveDirectAngle(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
          translationY.getAsDouble(),
          rotation.getAsDouble() * Math.PI,
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0);
  }

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

  // Drive a set amount of distance at a given speed
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return Commands.deferredProxy(
        () -> Commands.run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)), this)
            .until(
                () -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters));
  }

  public void setMaximumSpeed(double maximumSpeedInMetersPerSecond) {
    swerveDrive.setMaximumSpeed(maximumSpeedInMetersPerSecond,
        false,
        swerveDrive.swerveDriveConfiguration.physicalCharacteristics.optimalVoltage);
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  // The primary method for controlling the drivebase using the SwerveDrive class
  // It is recommended to use this method instead of the other two below.
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  // Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this method. 
  // However, if either gyro angle or module position is reset, this must called in order for odometry to keep working.
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  // Gets the current pose (position and rotation) of the robot, as reported by odometry. 
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // Set chassis speeds with closed-loop velocity control.
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  // Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // Checks if the alliance is red, defaults to false if alliance isn't available.
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  // Zero the robot to assume current position is facing forward
  // If red alliance, rotate the robot 180 after the drivebase zero command
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  // Sets the drive motors to brake/coast mode.
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  // Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
  // Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  // Get the chassis speeds based on controller input of 2 joysticks. 
  // One for speeds in which direction, the other for the angle of the robot.
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.Swerve.MAX_SPEED);
  }

  // Get the chassis speeds based on controller input of 1 joystick and one angle. 
  // Control the robot at an offset of 90deg.
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.Swerve.MAX_SPEED);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }
}

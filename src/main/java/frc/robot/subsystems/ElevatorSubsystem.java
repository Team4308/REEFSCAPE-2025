package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.constElevator;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorSubsystem extends SubsystemBase {

  public double MAX_HEIGHT = Units.inchesToMeters(50.0); //  (CHANGE VALUE TO REAL)

  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private double currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(constElevator.ELEVATOR_LEADER); 
    rightMotorLeader = new TalonFX(constElevator.ELEVATOR_FOLLOWER); 

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }




  public void setPosition(double setpointMeters) { // Set pos in meters 
    double setpointRotations = setpointMeters / (Math.PI * constElevator.SPOOL_RADIUS);
    double motorRotations = setpointRotations * constElevator.GEAR_RATIO;
    setPosition(motorRotations, currentVelocityLimit);

}

public void setPosition(double motorRotations, double velocityRPS) { // Default setPos in encoder units
  
    double currentMotorRotations = getPosition();
    double pidOutput = constElevator.pidController.calculate(currentMotorRotations, motorRotations);
    double feedforwardVoltage = constElevator.feedforward.calculate(velocityRPS);
    double totalVoltage = pidOutput + feedforwardVoltage;
    if (getPosition() ==  motorRotations) return; // (? Not needed? )
    rightMotorLeader.setVoltage(totalVoltage);
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
}
  // Preset position commands

  public Command goToLevel(int lvl) {
    System.out.println("Settings level to: " + lvl);
    switch (lvl) {
      case 0:
        return  this.runOnce(() -> setPosition(0.0));
      case 1:
        return  this.runOnce(() -> setPosition(constElevator.L1));
      case 2:
        return  this.runOnce(() -> setPosition(constElevator.L2));
      case 3:
        return  this.runOnce(() -> setPosition(constElevator.L3));
      case 4:
        return  this.runOnce(() -> setPosition(constElevator.L4));
      default:
        return null;
    }
  }


  // Speed control
  public void setNormalSpeed() {
    currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;
  }

  public void setSlowSpeed() {
    currentVelocityLimit = constElevator.SLOW_MOTOR_RPS;
  }

  public void setMaxSpeed() {
    currentVelocityLimit = constElevator.MAX_MOTOR_RPS;
  }

  public void setCustomSpeed(double speedMetersPerSecond) {
    currentVelocityLimit = speedMetersPerSecond * constElevator.GEAR_RATIO;
  }


  // Elevator data
  public double getPosition() {
    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble();
    return motorRotations / constElevator.GEAR_RATIO;  
  }

  public double getPositionInMeters() {
    return getPosition() * (Math.PI * constElevator.SPOOL_RADIUS);
  }

  public double getMaxHeight() {
    return MAX_HEIGHT;
  }


  // Stop the controllers
  public void stopControllers() {
    rightMotorLeader.set(0.0);
    leftMotorFollower.set(0.0);
  }

  // Reset the sensor position of the elevator
  public void resetSensorPosition(double setpoint) { 
    rightMotorLeader.setPosition(setpoint);
    leftMotorFollower.setPosition(setpoint);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position " , getPositionInMeters());
    SmartDashboard.putNumber("Elevator Max Height " , getMaxHeight());
    SmartDashboard.putNumber("Elevator Current " , rightMotorLeader.getStatorCurrent().getValueAsDouble());
    
  }
  public void simulationPeriodic()
  {
    if (Robot.isSimulation()) {
      TalonFXSimState talonFXSimRight = rightMotorLeader.getSimState();
      TalonFXSimState talonFXSimLeft = leftMotorFollower.getSimState();
      talonFXSimRight.setSupplyVoltage(RobotController.getBatteryVoltage());
      talonFXSimLeft.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
  }

  public Command homeElevator() {
    
    return run(() -> {
      // Go down until bottom limit
      rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_DOWN);
      leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    })
    .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD)
    .beforeStarting(() -> {
      setSlowSpeed();
      System.out.println("Starting elevator homing sequence...");
    })
    .andThen(() -> {
      stopControllers();
      resetSensorPosition(0.0);
      System.out.println("Found bottom, moving up...");
    })
    .andThen(
      run(() -> {
        // Go up until top limit
        rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_UP);
        leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
      })
      .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD)
    )
    .finallyDo((interrupted) -> {
      stopControllers();
      setNormalSpeed();
      if (!interrupted) {
        double foundMaxHeight = getPositionInMeters();
        MAX_HEIGHT = foundMaxHeight;
        System.out.println("Homing done. New max height: " + foundMaxHeight);

      } else {
        System.out.println("Homing sequence failed!");

      }
    });
  }

}


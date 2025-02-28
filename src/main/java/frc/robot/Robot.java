// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.control.XBoxWrapper;

public class Robot extends TimedRobot {

  public final TalonFX right;
  public final TalonFX left;

  public final XBoxWrapper button;

  final VelocityVoltage rightVelocity;
  final VelocityVoltage leftVelocity;

  button = new XBoxWrapper(0);

  public Robot() { 

  }

  @Override
  public void teleopInit() {

    right = new TalonFX(0);
    left = new TalonFX(1);

    rightVelocity = new VelocityVoltage(0);
        leftVelocity = new VelocityVoltage(0);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        right.getConfigurator().apply(rightConfiguration);
        left.getConfigurator().apply(leftConfiguration);

        right.getConfigurator().apply(slot0Configs, Constants.Generic.timeoutMs);
        left.getConfigurator().apply(slot0Configs, Constants.Generic.timeoutMs);

  }

  @Override
  public void teleopPeriodic() {

    double pressed1 = res(button.getRightY());
    double pressed2 = res(button.getLeftY());

    rightVelocity.Slot = 0;
        leftVelocity.Slot = 0;
        right.setControl(rightVelocity.withVelocity(15*pressed1));
        left.setControl(leftVelocity.withVelocity(15*pressed2));
  }

  double res(double num){
    if(num<0.05&&num>-0.05){
      return 0;
    }
    return num;
  }

  @Override
  public void teleopExit() {}

}

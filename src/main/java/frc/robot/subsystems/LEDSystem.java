package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;

public class LEDSystem extends SubsystemBase {
  private ElevatorSubsystem m_elevator;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private String led_status = "Idle";
  private int scrollOffset = 0;
  private static final int SCROLL_SPEED = 1;
  private static final int PATTERN_LENGTH = 2;

  public LEDSystem() {
    m_led = new AddressableLED(Constants.constLED.LED_PORT);
    m_buffer = new AddressableLEDBuffer(Constants.constLED.LED_LENGTH);
    m_led.setLength(Constants.constLED.LED_LENGTH);
    m_led.start();
  } 

  public void setElevator(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
  }

  public String getLedState() {
    return led_status;
  }


  public void setLedState(String status) {
    led_status = status;
  }

  @Override
  public void periodic() {
    switch (led_status) {
      case "Idle":
        for (int i = 0; i < m_buffer.getLength(); i++) {
          int patternPosition = (i + scrollOffset) % (PATTERN_LENGTH * 2);
          if (patternPosition < PATTERN_LENGTH) {
            m_buffer.setLED(i, Color.kRed);
          } else {
            m_buffer.setLED(i, Color.kWhite);
          }
        }
        scrollOffset = (scrollOffset + SCROLL_SPEED) % (PATTERN_LENGTH * 2);
        break;

      case "Auto":
        LEDPattern autoPattern = LEDPattern.solid(Color.kBlue);
        autoPattern.blink(Seconds.of(0.5));
        autoPattern.applyTo(m_buffer);
        break;

      case "Teleop":
        LEDPattern teleopBase = LEDPattern.solid(Color.kGreen);
        LEDPattern heightMask = LEDPattern.progressMaskLayer(() -> 
            m_elevator.getPositionInMeters() / m_elevator.getMaxHeight());
        teleopBase.mask(heightMask).applyTo(m_buffer);
        break;

      case "Test":
        LEDPattern testPattern = LEDPattern.solid(Color.kYellow);
        testPattern.blink(Seconds.of(0.2));
        testPattern.applyTo(m_buffer);
        break;

      case "Fault":
        LEDPattern.solid(Color.kRed).applyTo(m_buffer);
        break;

      case "Homing":
        LEDPattern homingPattern = LEDPattern.solid(Color.kOrange);
        homingPattern.breathe(Seconds.of(1.0));
        homingPattern.applyTo(m_buffer);
        break;

      case "Algae":
        LEDPattern algaePattern = LEDPattern.solid(Color.kGreen);
        algaePattern.breathe(Seconds.of(1.0));
        algaePattern.applyTo(m_buffer);
        break;

      case "Coral":
        // White breathing pattern
        LEDPattern coralPattern = LEDPattern.solid(Color.kWhite);
        coralPattern.breathe(Seconds.of(1.0));
        coralPattern.applyTo(m_buffer);
        break;

      case "Ready":
        // Rainbow pattern
        LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 60.0)).applyTo(m_buffer);;
        break;

      default:
        // Default progress bar with blue base
        LEDPattern defaultBase = LEDPattern.solid(Color.kBlue);
        LEDPattern defaultMask = LEDPattern.progressMaskLayer(() -> 
            m_elevator.getPositionInMeters() / m_elevator.getMaxHeight());
        defaultBase.mask(defaultMask).applyTo(m_buffer);
        break;
    }

    m_led.setData(m_buffer);
    SmartDashboard.putString("LED State", led_status);
  }
}
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
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

public class LEDSystem extends SubsystemBase {
  private ElevatorSubsystem m_elevator;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private String led_status = "Idle";
  private int scrollOffset = 0;
  private static final int SCROLL_SPEED = 1;
  private static final int PATTERN_LENGTH = 7; 
  private static final int TRAIL_LENGTH = 3;  

  private SimDevice m_simDevice;
  private SimDouble m_simR;
  private SimDouble m_simG;
  private SimDouble m_simB;
  private SimDouble m_simBrightness;

  public LEDSystem() {
    m_led = new AddressableLED(Constants.constLED.LED_PORT);
    m_buffer = new AddressableLEDBuffer(Constants.constLED.LED_LENGTH);
    m_led.setLength(Constants.constLED.LED_LENGTH);
    m_led.start();

    m_simDevice = SimDevice.create("LED_System");
    if (m_simDevice != null) {
      m_simR = m_simDevice.createDouble("R", SimDevice.Direction.kOutput, 0.0);
      m_simG = m_simDevice.createDouble("G", SimDevice.Direction.kOutput, 0.0);
      m_simB = m_simDevice.createDouble("B", SimDevice.Direction.kOutput, 0.0);
      m_simBrightness = m_simDevice.createDouble("Brightness", SimDevice.Direction.kOutput, 0.0);
    }
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
    // led_status = "";
    switch (led_status) {
      case "Idle":
        for (int i = 0; i < m_buffer.getLength(); i++) {
          

          
          int position = (i + scrollOffset) % (PATTERN_LENGTH * 3); 
          
          if (position < PATTERN_LENGTH) {
            double fade = getFadeValue(position, PATTERN_LENGTH);
            m_buffer.setLED(i, new Color(fade * 1.0, 0, 0));
          } 
          else if (position < PATTERN_LENGTH * 2) {
            double fade = getFadeValue(position - PATTERN_LENGTH, PATTERN_LENGTH);
            m_buffer.setLED(i, new Color(1.0, fade, fade));
          }
          else {
            double fade = 1.0 - getFadeValue(position - (PATTERN_LENGTH * 2), PATTERN_LENGTH);
            m_buffer.setLED(i, new Color(fade, fade, fade));
          
        }
      }
        scrollOffset = (scrollOffset + SCROLL_SPEED ) % (PATTERN_LENGTH * 3);
        break;

      case "Auto":
        LEDPattern basePattern = LEDPattern.rainbow(180, 240)  
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(1.0/60.0));
                LEDPattern maskPattern = LEDPattern.rainbow(180, 240)  
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.3), Meters.of(1.0/60.0));
        
        basePattern.mask(maskPattern)
            .blink(Seconds.of(0.5))
            .applyTo(m_buffer);
        break;

      case "Teleop":
        LEDPattern teleopBase = LEDPattern.rainbow(255, 128)  
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.7), Meters.of(1.0/60.0));
        
        LEDPattern heightMask = LEDPattern.progressMaskLayer(() -> 
            m_elevator.getPositionInMeters() - 0.1 / m_elevator.getMaxHeight());  
        
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
// One color for Idle (In game), One color for when not allgined for anything but has coral, Another color for Allinged to the reef (Doesnt matter iwth or without coral ) While allginng blink while not soild green 
      case "Coral":
        LEDPattern coralPattern = LEDPattern.solid(Color.kWhite);
        coralPattern.breathe(Seconds.of(1.0));
        coralPattern.applyTo(m_buffer);
        break;

      case "Ready":
        LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 60.0)).applyTo(m_buffer);;
        break;

      default:
        LEDPattern defaultBase = LEDPattern.solid(Color.kBlue);
        LEDPattern defaultMask = LEDPattern.progressMaskLayer(() -> 
            m_elevator.getPositionInMeters() / m_elevator.getMaxHeight());
        defaultBase.mask(defaultMask).applyTo(m_buffer);
        break;
    }

    m_led.setData(m_buffer);
    SmartDashboard.putString("LED State", led_status);

    if (m_simDevice != null) {
      double totalR = 0, totalG = 0, totalB = 0;
      for (int i = 0; i < m_buffer.getLength(); i++) {
        Color color = m_buffer.getLED(i);
        totalR += color.red;
        totalG += color.green;
        totalB += color.blue;
      }
      
      double length = m_buffer.getLength();
      m_simR.set(totalR / length);
      m_simG.set(totalG / length);
      m_simB.set(totalB / length);      m_simBrightness.set(Math.max(Math.max(totalR, totalG), totalB) / length);    }  }
  private double getFadeValue(int position, int patternLength) {
    if (position < TRAIL_LENGTH) {
      return (double)position / TRAIL_LENGTH;
    } else if (position >= patternLength - TRAIL_LENGTH) {
      return (double)(patternLength - position) / TRAIL_LENGTH;
    }
    return 1.0;
  }
}
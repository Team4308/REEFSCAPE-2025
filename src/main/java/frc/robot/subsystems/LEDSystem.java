package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class LEDSystem extends SubsystemBase {
  private ElevatorSubsystem m_elevator;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private String led_status = "Idle";
  public String robot_status = "A";
  private int scrollOffset = 0;
  private Color statusColor;
  private double statusLightTimer = 0;
  private boolean showingStatus = false;
 

  private SimDevice m_simDevice;
  private SimDouble m_simR;
  private SimDouble m_simG;
  private SimDouble m_simB;
  private SimDouble m_simBrightness;

  public LEDSystem() {
    m_led = new AddressableLED(Constants.constLED.LED_PORT);
    m_buffer = new AddressableLEDBuffer(Constants.constLED.LED_LENGTH);
    
    try {
      m_led.setLength(Constants.constLED.LED_LENGTH);
      m_led.start();
    } catch (Exception e) {
      System.err.println("Error with LED strip: " + e.getMessage());
      e.printStackTrace();
    }
    
    // Sim Setup
    m_simDevice = SimDevice.create("LED_System");
    if (m_simDevice != null) {
      m_simR = m_simDevice.createDouble("R", SimDevice.Direction.kOutput, 0.0);
      m_simG = m_simDevice.createDouble("G", SimDevice.Direction.kOutput, 0.0);
      m_simB = m_simDevice.createDouble("B", SimDevice.Direction.kOutput, 0.0);
      m_simBrightness = m_simDevice.createDouble("Brightness", SimDevice.Direction.kOutput, 0.0);
  } 
  } 

  /**
   *  Sets the elevator var in LED's when its initialized
   * @param elevator
   */
  public void setElevator(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
  }
  /**
   * Returns the current state of the LED's
   * @return led_status
   */
  public String getLedState() {
    return led_status;
  }

 /**
  * Sets the state of the LED's
  * @param status
  */
  public void setLedState(String status) {
    led_status = status;
  }


  @Override
  public void periodic() {
  
    applyPattern(led_status);
    m_led.setData(m_buffer);
    updateSimulation();
    
  }


  /**
   * Direcly applys and updated the buffer of the leds
   * @param state
   */

  private void applyPattern(String state) {
    switch (state) {
      case "Idle":
        for (int i = 0; i < Constants.constLED.LED_LENGTH; i++) {
          int position = (i + scrollOffset) % (constLED.PATTERN_LENGTH * 3);
          setIdlePattern(m_buffer, i, position);
        }
        scrollOffset = (scrollOffset + constLED.SCROLL_SPEED) % (constLED.PATTERN_LENGTH * 3);
        break;
        
      case "Auto":
        LEDPattern basePattern = LEDPattern.rainbow(180, 240)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), Units.Meters.of(1.0/constLED.LED_LENGTH));
        LEDPattern maskPattern = LEDPattern.rainbow(180, 240)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.3), Units.Meters.of(1.0/constLED.LED_LENGTH));
        basePattern.mask(maskPattern).blink(Units.Seconds.of(0.5)).applyTo(m_buffer);
        break;

      case "Teleop":
      LEDPattern teleopBase = LEDPattern.rainbow(255, 128)  
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.7), Meters.of(1.0/constLED.LED_LENGTH));
        LEDPattern heightMask = LEDPattern.progressMaskLayer(() -> 
           m_elevator.getPositionInMeters() - 0.1 / m_elevator.getMaxHeight());  
        teleopBase.mask(heightMask).applyTo(m_buffer);
      break;

      // Test state of Robot idk when this is used tbh
      case "Aligned":
        LEDPattern.solid(Color.kGreen).applyTo(m_buffer);
      break;

      case "Test":
        LEDPattern.solid(Color.kYellow)
            .blink(Units.Seconds.of(0.2))
            .applyTo(m_buffer);
        break;

        // only called on fault from roborio this is if somthing really bad happens
      case "Fault":
        LEDPattern.solid(Color.kRed).applyTo(m_buffer);
        break;

      default:
        LEDPattern.solid(Color.kBlue)
            .mask(LEDPattern.progressMaskLayer(() -> 
                m_elevator.getPositionInMeters() / m_elevator.getMaxHeight()))
            .applyTo(m_buffer);
        break;
    }
  }


  // Hade to create a helper function 
  private void setIdlePattern(AddressableLEDBuffer buffer, int i, int position) {
    if (position < constLED.PATTERN_LENGTH) {
      double fade = getFadeValue(position, constLED.PATTERN_LENGTH);
      buffer.setLED(i, new Color(fade * 1.0, 0, 0));
    } else if (position < constLED.PATTERN_LENGTH * 2) {
      double fade = getFadeValue(position - constLED.PATTERN_LENGTH, constLED.PATTERN_LENGTH);
      buffer.setLED(i, new Color(1.0, fade, fade));
    } else {
      double fade = 1.0 - getFadeValue(position - (constLED.PATTERN_LENGTH * 2), constLED.PATTERN_LENGTH);
      buffer.setLED(i, new Color(fade, fade, fade));
    }
  }

  /** Update Simulation
   * @brief Updates the simulation values for the LED strip
   * 
   */
  private void updateSimulation() {
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
      m_simB.set(totalB / length);
      m_simBrightness.set(Math.max(Math.max(totalR, totalG), totalB) / length);
    }
  }
/**
 * getFadeValue
 * @brief Returns the fade value for the LED strip
 * @param position
 * @param patternLength
 * @return Double
*/
  private double getFadeValue(int position, int patternLength) {
    if (position < constLED.TRAIL_LENGTH) {
      return (double)position / constLED.TRAIL_LENGTH;
    } else if (position >= patternLength - constLED.TRAIL_LENGTH) {
      return (double)(patternLength - position) / constLED.TRAIL_LENGTH;
    }
    return 1.0;
  }
}
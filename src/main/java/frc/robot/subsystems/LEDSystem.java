package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class LEDSystem extends SubsystemBase {
  private ElevatorSubsystem m_elevator;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final RobotContainer robotContainer;

  private int scrollOffset = 0;

  private SimDevice m_simDevice;
  private SimDouble m_simR;
  private SimDouble m_simG;
  private SimDouble m_simB;
  private SimDouble m_simBrightness;

  public String previousState = "";
  public String currentState = "";
  private String baseState = ""; // Add this to track the underlying state
  private boolean isShowingStatus = false;
  private double statusTimer = 0;
  private static final double STATUS_DURATION = 2.8;

  public LEDSystem(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
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
   * Sets the elevator var in LED's when its initialized
   * 
   * @param elevator
   */
  public void setElevator(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
  }

  /**
   * Returns the current state of the LED's
   * 
   * @return led_status
   */
  public String getLedState() {
    return currentState;
  }

  /**
   * Sets the state of the LED's
   * 
   * @param status
   */
  public void setLedState(String status) {
    if (status == null || status.trim().isEmpty()) {
      return;
    }

    // Store base state for non-status states
    if (!status.equals("Aligned") && !status.equals("Fault")) {
      baseState = status;
    }

    if (!status.equals(currentState)) {
      previousState = currentState;
      currentState = status;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("currentState", currentState);
    SmartDashboard.putString("Last LED status", previousState);
    SmartDashboard.putString("Base State", baseState); // Debug output

    // Skip if no valid state
    if (currentState == null || currentState.trim().isEmpty()) {
      return;
    }

    if (isShowingStatus) {
      statusTimer -= 0.02;
      if (statusTimer <= 0) {
        isShowingStatus = false;
        currentState = baseState; // Return to base state instead of previous state
      }
    } else if (currentState.equals("Aligned") || currentState.equals("Fault")) {
      isShowingStatus = true;
      statusTimer = STATUS_DURATION;
    }

    applyPattern(currentState);
    m_led.setData(m_buffer);
    updateSimulation();
  }

  /**
   * Direcly applys and updated the buffer of the leds
   * 
   * @param state
   */

  private void applyPattern(String state) {
    // Show brief status indication by modifying the normal patterns
    if (isShowingStatus) {
      switch (state) {
        case "Aligned":
          LEDPattern.solid(Color.kGreen)
              .blink(Units.Seconds.of(0.2))
              .applyTo(m_buffer);
          return;
        case "Fault":
          LEDPattern.solid(Color.kRed)
              .blink(Units.Seconds.of(0.2))
              .applyTo(m_buffer);
          return;
      }
    }

    // Normal pattern handling
    switch (state) {
      case "Idle":
        for (int i = 0; i < Constants.constLED.LED_LENGTH; i++) {
          int position = (i + scrollOffset) % (constLED.PATTERN_LENGTH * 3);
          Optional<Alliance> alliance = DriverStation.getAlliance();
          double r = 1.0, g = 0.0, b = 0.0; // Default to red

          if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            r = 0.0;
            g = 0.0;
            b = 1.0;
          }

          setIdlePattern(m_buffer, i, position, r, g, b);
        }
        scrollOffset = (scrollOffset + constLED.SCROLL_SPEED) % (constLED.PATTERN_LENGTH * 3);
        break;

      case "Auto":
        LEDPattern basePattern = LEDPattern.rainbow(180, 240)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), Units.Meters.of(1.0 / constLED.LED_LENGTH));
        LEDPattern maskPattern = LEDPattern.rainbow(180, 240)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.3), Units.Meters.of(1.0 / constLED.LED_LENGTH));
        basePattern.mask(maskPattern).blink(Units.Seconds.of(0.5)).applyTo(m_buffer);
        break;

      case "Teleop":
        LEDPattern teleopBase = LEDPattern.rainbow(255, 128)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.7), Meters.of(1.0 / constLED.LED_LENGTH));
        LEDPattern heightMask = LEDPattern
            .progressMaskLayer(() -> m_elevator.getPositionInMeters() - 0.1 / constElevator.MAX_HEIGHT);
        teleopBase.mask(heightMask).applyTo(m_buffer);
        break;

      case "Aligned":
        LEDPattern.solid(Color.kGreen).applyTo(m_buffer);
        break;

      case "Coral":
        LEDPattern.solid(Color.kYellow).applyTo(m_buffer);
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
            .mask(LEDPattern.progressMaskLayer(() -> m_elevator.getPositionInMeters() / constElevator.MAX_HEIGHT))
            .applyTo(m_buffer);
        break;
    }
  }

  /**
   * Sets the LEDS to be a fading value from given (R,G,B) to White
   * 
   * @param buffer   Led Buffer
   * @param i        Led index
   * @param position Current Pos
   * @param baseR    Red
   * @param baseG    Green
   * @param baseB    Blue
   */
  private void setIdlePattern(AddressableLEDBuffer buffer, int i, int position, double baseR, double baseG,
      double baseB) {
    if (position < constLED.PATTERN_LENGTH) {
      double fade = getFadeValue(position, constLED.PATTERN_LENGTH);
      double blendFactor = fade * 0.3;
      buffer.setLED(i, new Color(
          baseR + (blendFactor * (1.0 - baseR)),
          baseG + (blendFactor * (1.0 - baseG)),
          baseB + (blendFactor * (1.0 - baseB))));
    } else if (position < constLED.PATTERN_LENGTH * 2) {
      double fade = getFadeValue(position - constLED.PATTERN_LENGTH, constLED.PATTERN_LENGTH);
      double blendFactor = 0.3 + (fade * 0.4);
      buffer.setLED(i, new Color(
          baseR + (blendFactor * (1.0 - baseR)),
          baseG + (blendFactor * (1.0 - baseG)),
          baseB + (blendFactor * (1.0 - baseB))));
    } else {
      double fade = getFadeValue(position - (constLED.PATTERN_LENGTH * 2), constLED.PATTERN_LENGTH);
      double blendFactor = 0.7 + (fade * 0.3);
      buffer.setLED(i, new Color(
          baseR + (blendFactor * (1.0 - baseR)),
          baseG + (blendFactor * (1.0 - baseG)),
          baseB + (blendFactor * (1.0 - baseB))));
    }
  }

  /**
   * Update Simulation
   * 
   * @brief Updates the simulation values for the LED strip
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
   * 
   * @brief Returns the fade value for the LED strip
   * @param position
   * @param patternLength
   * @return Double
   */
  private double getFadeValue(int position, int patternLength) {
    if (position < constLED.TRAIL_LENGTH) {
      return (double) position / constLED.TRAIL_LENGTH;
    } else if (position >= patternLength - constLED.TRAIL_LENGTH) {
      return (double) (patternLength - position) / constLED.TRAIL_LENGTH;
    }
    return 1.0;
  }
}
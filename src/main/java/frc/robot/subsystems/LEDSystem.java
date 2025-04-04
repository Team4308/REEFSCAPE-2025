package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

/*
 * three types of states:
 *    default(idle, teleop, auton)
 *    temporary, runs continusly while true(aligned)
 *    status, flashes once(intake)
 */

public class LEDSystem extends SubsystemBase {
  private ElevatorSubsystem m_elevator;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final AddressableLEDBufferView m_elevatorBuffer;
  private final AddressableLEDBufferView m_funnelVertBuffer;
  private final AddressableLEDBufferView m_funnelHoriBuffer;

  private int scrollOffsetElevator = 0;
  private int scrollOffsetVertFunnel = 0;

  private SimDevice m_simDevice;
  private SimDouble m_simR;
  private SimDouble m_simG;
  private SimDouble m_simB;
  private SimDouble m_simBrightness;

  public HashMap<String, String> stateCarrier;
  private boolean isShowingStatus = false;
  private double statusTimer = 0;

  private HashMap<String, LEDTuple> states;

  public LEDSystem(RobotContainer robotContainer) {
    m_led = new AddressableLED(constLED.LED_PORT);
    m_buffer = new AddressableLEDBuffer(constLED.LED_LENGTH);
    m_elevatorBuffer = m_buffer.createView(constLED.Elevator_Ends.getFirst(), constLED.Elevator_Ends.getSecond());
    m_funnelVertBuffer = m_buffer.createView(constLED.Funnel_Vert_Ends.getFirst(),
        constLED.Funnel_Vert_Ends.getSecond());
    m_funnelHoriBuffer = m_buffer.createView(constLED.Funnel_Hori_Ends.getFirst(),
        constLED.Funnel_Hori_Ends.getSecond());

    try {
      m_led.setLength(constLED.LED_LENGTH);
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

    stateCarrier = new HashMap<String, String>();
    stateCarrier.put("DefaultState", null);
    stateCarrier.put("TemporaryState", null);
    stateCarrier.put("StatusState", null);

    states = new HashMap<String, LEDTuple>();
    states.put("Coral", new LEDTuple(true, 2.0, "StatusState"));
    states.put("Aligned", new LEDTuple(false, 0.0, "TemporaryState"));
    states.put("Fault", new LEDTuple(true, 2.0, "StatusState"));
    states.put("Idle", new LEDTuple(false, 0.0, "DefaultState"));
    states.put("Auto", new LEDTuple(false, 0.0, "DefaultState"));
    states.put("Teleop", new LEDTuple(false, 0.0, "DefaultState"));
    states.put("Test", new LEDTuple(false, 0.0, "DefaultState"));
    states.put("CoralShot", new LEDTuple(true, 0.0, "StatusState"));
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
    String currentState = null;
    for (String stateType : new ArrayList<String>() {
      {
        add("StatusState");
        add("TemporaryState");
        add("DefaultState");
      }
    }) {
      currentState = stateCarrier.get(stateType);

      if (currentState == null || currentState.trim().isEmpty()) {
        continue;
      }
    }
    return currentState;
  }

  /**
   * Sets the state of the LED's
   * 
   * @param status
   */
  public void setLedState(String status) {
    if (status == null || status.trim().isEmpty() || !states.containsKey(status)) {
      return;
    }

    String stateType = states.get(status).stateType;

    System.out.print("Changing state to: ");
    System.out.println(status);
    stateCarrier.replace(stateType, status);

  }

  @Override
  public void periodic() {
    // String defaultState = stateCarrier.get("DefaultState") == null ? "None" :
    // stateCarrier.get("DefaultState");
    // String tempState = stateCarrier.get("TemporaryState") == null ? "None" :
    // stateCarrier.get("TemporaryState");
    // String statState = stateCarrier.get("StatusState") == null ? "None" :
    // stateCarrier.get("StatusState");
    // SmartDashboard.putString("Default State:", defaultState);
    // SmartDashboard.putString("Temporary State", tempState);
    // SmartDashboard.putString("Status State", statState); // Debug output

    String currentState = null;
    // starts in order from status, temp, default
    for (String stateType : new ArrayList<String>() {
      {
        add("StatusState");
        add("TemporaryState");
        add("DefaultState");
      }
    }) {
      currentState = stateCarrier.get(stateType);
      if (currentState != null) {
        break;
      }
    }
    if (currentState == null || currentState.trim().isEmpty()) {
      return;
    }

    if (isShowingStatus) {
      statusTimer -= 0.02;
      if (statusTimer <= 0) {
        stateCarrier.replace("StatusState", null);
        isShowingStatus = false;
      }
    } else {
      isShowingStatus = states.get(currentState).getState();
      statusTimer = 0.0;
      if (isShowingStatus)
        statusTimer = states.get(currentState).getDuration();
    }

    applyState(currentState);
    m_led.setData(m_buffer);
    updateSimulation();

    Logger.recordOutput("Subsystems/LED", m_buffer.toString());

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

  public void clearStatus() {
    stateCarrier.replace("StatusState", null);
  }

  public void clearTemporary() {
    stateCarrier.replace("TemporaryState", null);
  }

  /**
   * Direcly applys and updated the buffer of the leds
   * 
   * @param state
   */

  private void applyState(String state) {
    switch (state) {
      case "Idle":
        for (int i = 0; i < constLED.Elevator_Length; i++) {
          int position = (i + scrollOffsetElevator) % (constLED.PATTERN_LENGTH * 3);
          Optional<Alliance> alliance = DriverStation.getAlliance();
          double r = 1.0, g = 0.0, b = 0.0; // Default to red
          if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            r = 0.0;
            g = 0.0;
            b = 1.0;
          }
          setIdlePattern(m_elevatorBuffer, i, position, r, g, b);
        }
        scrollOffsetElevator = (scrollOffsetElevator + constLED.SCROLL_SPEED) % (constLED.PATTERN_LENGTH * 3);

        for (int i = 0; i < constLED.Funnel_Vert_Length; i++) {
          int position = (i + scrollOffsetVertFunnel) % (constLED.PATTERN_LENGTH * 3);
          Optional<Alliance> alliance = DriverStation.getAlliance();
          double r = 1.0, g = 0.0, b = 0.0; // Default to red
          if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            r = 0.0;
            b = 1.0;
          }
          setIdlePattern(m_funnelVertBuffer, i, position, r, g, b);
        }
        scrollOffsetVertFunnel = (scrollOffsetVertFunnel + constLED.SCROLL_SPEED) % (constLED.PATTERN_LENGTH * 3);

        Optional<Alliance> alliance = DriverStation.getAlliance();
        double r = 1.0, g = 0.0, b = 0.0; // Default to red
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
          r = 0.0;
          b = 1.0;
        }

        LEDPattern base = LEDPattern.solid(new Color(r, g, b));
        LEDPattern pattern = base.breathe(Units.Seconds.of(2));

        // Apply the LED pattern to the data buffer
        pattern.applyTo(m_funnelHoriBuffer);

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
        teleopBase.mask(heightMask).applyTo(m_elevatorBuffer);

        Optional<Alliance> alliance2 = DriverStation.getAlliance();
        double r2 = 1.0, g2 = 0.0, b2 = 0.0; // Default to red
        if (alliance2.isPresent() && alliance2.get() == Alliance.Blue) {
          r2 = 0.0;
          b2 = 1.0;
        }
        LEDPattern base2 = LEDPattern.solid(new Color(r2, g2, b2));
        LEDPattern pattern2 = base2.breathe(Units.Seconds.of(2));

        // Apply the LED pattern to the data buffer
        pattern2.applyTo(m_funnelVertBuffer);

        base2 = LEDPattern.solid(new Color(r2, g2, b2));
        pattern2 = base2.breathe(Units.Seconds.of(2));

        // Apply the LED pattern to the data buffer
        pattern2.applyTo(m_funnelHoriBuffer);

        break;

      case "Aligned":
        LEDPattern.solid(Color.kGreen)
            .blink(Units.Seconds.of(0.2))
            .applyTo(m_buffer);
        break;

      case "Coral":
        LEDPattern.solid(Color.kYellow)
            .blink(Units.Seconds.of(0.2))
            .applyTo(m_buffer);

        return;

      case "Test":
        LEDPattern.solid(Color.kPurple)
            .blink(Units.Seconds.of(0.2))
            .applyTo(m_buffer);
        break;

      // only called on fault from roborio this is if somthing really bad happens
      case "Fault":
        LEDPattern.solid(Color.kRed)
            .blink(Units.Seconds.of(0.2))
            .applyTo(m_buffer);
        return;

      case "CoralShot":
        LEDPattern.solid(Color.kOrange)
            .blink(Units.Seconds.of(0.2))
            .applyTo(m_buffer);
        return;

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
  private void setIdlePattern(AddressableLEDBufferView buffer, int i, int position, double baseR, double baseG,
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

  public class LEDTuple {
    public final Boolean isTemporary;
    public final Double statusDuration;
    public final String stateType;

    public LEDTuple(Boolean isTemporary, Double statusDuration, String stateType) {
      this.isTemporary = isTemporary;
      this.statusDuration = statusDuration;
      this.stateType = stateType;
    }

    public Boolean getState() {
      return this.isTemporary;
    }

    public Double getDuration() {
      return this.statusDuration;
    }

    public String getType() {
      return this.stateType;
    }
  }
}
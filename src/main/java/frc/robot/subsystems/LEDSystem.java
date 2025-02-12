package frc.robot.subsystems;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class LEDSystem extends SubsystemBase {
  private ElevatorSubsystem m_elevator;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private String ledState = "Idle";
  private Color currentColor = Color.kBlack;
  private double patternValue = 0.0;

  public LEDSystem() {
    m_led = new AddressableLED(Constants.constLED.LED_PORT);
    m_buffer = new AddressableLEDBuffer(Constants.constLED.LED_LENGTH);
    m_led.setLength(Constants.constLED.LED_LENGTH);
    m_led.start();

    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Idle")); // Set it to default 

    // Initialize simulation
    if (Robot.isSimulation()) {
        SmartDashboard.putString("LED State", ledState);
        SmartDashboard.putNumber("LED Pattern Value", 0.0);
        SmartDashboard.putNumber("LED R", 0.0);
        SmartDashboard.putNumber("LED G", 0.0);
        SmartDashboard.putNumber("LED B", 0.0);
    }
  }

  public void setElevator(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
  }

  public String getLedState() {
    return ledState;  
  }

  // Color Map 

  // Red Fault, Error, or other critical issuse 
  // Yellow, Test, Debug, Etc
  // Orange Zeroing, Calibrating, etc ( Do not interrupt )

  // Green, Game Peice
  // White Game Peice  

  // Blinking Auto (Any Color)
  // Anything else just Teleop (MaskedLayer , Graident, Rainbow, etc)

  public void setLedState(String status) {  
    System.out.println("Setting LED State to: " + status);
    ledState = status;
    LEDPattern pattern;
    switch (status) {
      case "Idle": 
        currentColor = Color.kDimGray;
        runPattern(LEDPattern.solid(currentColor));
        break;
      case "Auto":
        currentColor = Color.kOrangeRed;
        pattern = LEDPattern.progressMaskLayer(() -> {
          patternValue = m_elevator.getHeight() / m_elevator.getMaxHeight();
          return patternValue;
        });
        pattern.blink(Units.Seconds.of(1));
        runPattern(pattern);
        break;
      case "Teleop":
        runPattern(LEDPattern.progressMaskLayer(() -> m_elevator.getHeight() / m_elevator.getMaxHeight()));
        break;
      case "Test":
        runPattern(LEDPattern.solid(Color.kYellow));
        break;
      case "Fault":
        runPattern(LEDPattern.solid(Color.kRed));
        break;
      case "Zeroing":
        runPattern(LEDPattern.solid(Color.kOrange));
        break;
      case "Algae":
        runPattern(LEDPattern.solid(Color.kSeaGreen));
        break;
      case "Coral":
        runPattern(LEDPattern.solid(Color.kWhiteSmoke));  
        break;
      default:
        // Idk if this works 
        runPattern(LEDPattern.progressMaskLayer(() -> m_elevator.getHeight() / m_elevator.getMaxHeight()));
        break;
    }
    
    if (Robot.isSimulation()) {
        SmartDashboard.putString("LED State", ledState);
        SmartDashboard.putNumber("LED R", currentColor.red);
        SmartDashboard.putNumber("LED G", currentColor.green);
        SmartDashboard.putNumber("LED B", currentColor.blue);
    }
  }

  @Override
  public void periodic() {
    m_led.setData(m_buffer);
    
    if (Robot.isSimulation()) {
        // Update pattern value for progress indicators
        if (ledState.equals("Auto") || ledState.equals("Teleop")) {
            patternValue = m_elevator.getHeight() / m_elevator.getMaxHeight();
            SmartDashboard.putNumber("LED Pattern Value", patternValue);
        }
        
        // Force dashboard updates
        SmartDashboard.putString("LED State", ledState);
        SmartDashboard.putNumber("LED R", currentColor.red);
        SmartDashboard.putNumber("LED G", currentColor.green);
        SmartDashboard.putNumber("LED B", currentColor.blue);
    }
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}
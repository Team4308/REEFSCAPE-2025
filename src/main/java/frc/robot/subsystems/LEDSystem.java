package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class LEDSystem extends SubsystemBase  {
  private static final int kPort = 9; // Adjust to Leds port 
  private static final int kLength = 120; // Adjust to Length (Todo: move to contants)
  private final ElevatorSubsystem m_elevator;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public LEDSystem(ElevatorSubsystem elevator) {
      this.m_elevator = elevator;
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("None")); // Set it to tge default
  }

  public void setLedState(String status) {
    switch (status) {
      case "Idle":
        runPattern(LEDPattern.solid(Color.kDimGray));
        break;
      case "Auto":
        runPattern(LEDPattern.solid(Color.kOrangeRed));
        break;
      case "Teleop":
        runPattern(LEDPattern.solid(Color.kCadetBlue));
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
      
        runPattern(LEDPattern.progressMaskLayer(() -> m_elevator.getHeight() / m_elevator.getMaxHeight()));
        break;
    }
  }

  @Override
  public void periodic() {

    m_led.setData(m_buffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}
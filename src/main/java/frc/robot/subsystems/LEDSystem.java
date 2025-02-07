package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSystem extends SubsystemBase  {
  private static final int kPort = 9;
  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public LEDSystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  public void setLedState(String status) {
    switch (status) {
        
      case "Off":
        runPattern(LEDPattern.solid(Color.kBlack));
        break;
      case "Idle":
        runPattern(LEDPattern.solid(Color.kWhite));
        break;
      case "Auto":
        runPattern(LEDPattern.solid(Color.kBlue));
        break;
      case "Teleop":
        runPattern(LEDPattern.solid(Color.kGreen));
        break;
      case "Test":
        runPattern(LEDPattern.solid(Color.kYellow));
        break;
      case "Fault":
        runPattern(LEDPattern.solid(Color.kRed));
        break;
      default:
        runPattern(LEDPattern.rainbow(0, 120));
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
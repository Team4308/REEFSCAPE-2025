package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.constLED;

import java.util.Optional;

import ca.team4308.absolutelib.leds.AddressableLEDBufferView;
import ca.team4308.absolutelib.leds.LEDConstants;
import ca.team4308.absolutelib.leds.LEDPattern;
import ca.team4308.absolutelib.leds.Leds;
import ca.team4308.absolutelib.leds.Patterns;
import ca.team4308.absolutelib.leds.TimeBasedLEDPattern;
import ca.team4308.absolutelib.leds.Utils;

/**
 * Ultra-simplified LED system that focuses on reliability
 */
public class LEDSystem extends SubsystemBase {
    
    // Hardware components
    public final Leds leds;
    private AddressableLEDBufferView mainView;
    
    // State variables
    private String state = "default";
    double startTime;
    private int counter = 0;
    private LEDPattern currentPattern = null;
    
    public LEDSystem() {
        leds = new Leds(constLED.LED_PORT, constLED.LED_LENGTH, true);
        leds.start();
        mainView = leds.createRangeBufferView(0, constLED.LED_LENGTH);
        startTime = Timer.getFPGATimestamp();
        currentPattern = Patterns.teamFlagWithFade();
        System.out.println("LED System initialized with " + constLED.LED_LENGTH + " LEDs on port " + constLED.LED_PORT);
    }
    
    @Override
    public void periodic() {
        counter++;
        
        try {
            updatePatternState();
            
            if (currentPattern != null) {
                currentPattern.applyTo(mainView);
                leds.update();
            }
                SmartDashboard.putString("LED/State", state);
                SmartDashboard.putNumber("LED/Counter", counter);
                Color c = mainView.getLED(0);
                SmartDashboard.putNumber("LED/FirstPixel_R", c.red);
                SmartDashboard.putNumber("LED/FirstPixel_G", c.green);
                SmartDashboard.putNumber("LED/FirstPixel_B", c.blue);
            
        } catch (Exception e) {
            System.err.println("LED update error: " + e.getMessage());
            setEmergencyPattern();
        }
    }
    
    /**
     * Updates the pattern based on current state
     */
    private void updatePatternState() {
        // Get current pattern based on state
        LEDPattern newPattern = null;
        
        switch (state) {
            case "rainbow":
                newPattern = Patterns.rainbowChase();
                break;
                
            case "teamflag":
                newPattern = Patterns.teamFlag();
                break;
                
            case "altf4":
                newPattern = Patterns.altF4();
                break;
                
            case "solid_red":
                newPattern = Utils.createSolidPattern(new Color(1, 0, 0));
                break;
                
            case "solid_blue":
                newPattern = Utils.createSolidPattern(new Color(0, 0, 1));
                break;
                
            case "solid_green":
                newPattern = Utils.createSolidPattern(new Color(0, 1, 0));
                break;
                
            case "blink":
                newPattern = Patterns.createBlinkingPattern(new Color(1, 0, 0), 0.5);
                break;
                
            case "breathe":
                newPattern = Utils.createBreathingPattern(new Color(1, 0, 0), 2.0);
                break;
                
            case "test":
                newPattern = createSimpleTestPattern();
                break;
                
            case "idle":
            case "default":
            default:
                LEDConstants.setPatternLength(10);
                SmartDashboard.putString("LED RGB", Utils.getAllianceColor().toString());
                newPattern = Patterns.scrollingIdle(Utils.getAllianceColor(),Color.kWhite, 1);

                break;
        }
        
        if (currentPattern == null || !isSamePatternType(currentPattern, newPattern)) {
            System.out.println("Changing LED pattern to " + state);
            
            if (newPattern instanceof TimeBasedLEDPattern) {
                ((TimeBasedLEDPattern) newPattern).resetTimer();
            }

            currentPattern = newPattern;
        }
    }
    
    /**
     * Checks if two patterns are of the same type
     */
    private boolean isSamePatternType(LEDPattern a, LEDPattern b) {
        if (a == null || b == null) return false;
        return a.getClass() == b.getClass();
    }
    
    /**
     * Sets the LED state
     */
    public void setLedState(String newState) {
        if (!newState.equals(state)) {
            System.out.println("LED state changing from " + state + " to " + newState);
            state = newState;
            startTime = Timer.getFPGATimestamp(); // Reset animation timing
            
            // Immediately update the pattern
            updatePatternState();
        }
    }
    
    /**
     * Gets current LED state
     */
    public String getLedState() {
        return state;
    }
    
    /**
     * Sets a simple emergency pattern in case of errors
     */
    private void setEmergencyPattern() {
        try {
            // Apply a simple red pattern
            for (int i = 0; i < constLED.LED_LENGTH; i++) {
                if (i % 2 == 0) {
                    mainView.setRGB(i, 255, 0, 0);
                } else {
                    mainView.setRGB(i, 0, 0, 0);
                }
            }
            leds.update();
        } catch (Exception ex) {
            // If even the emergency pattern fails, just log it
            System.err.println("Critical LED error: " + ex.getMessage());
        }
    }
    
    /**
     * Creates a simple test pattern with fixed colors
     */
    private LEDPattern createSimpleTestPattern() {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                // Use very bright colors for maximum visibility
                int length = view.getLength();
                for (int i = 0; i < length; i++) {
                    int section = (i * 4) / length;
                    
                    switch (section) {
                        case 0:
                            view.setRGB(i, 255, 0, 0);  // Red
                            break;
                        case 1:
                            view.setRGB(i, 0, 255, 0);  // Green
                            break;
                        case 2:
                            view.setRGB(i, 0, 0, 255);  // Blue
                            break;
                        case 3:
                        default:
                            view.setRGB(i, 255, 255, 0); 
                            break;
                    }
                }
                
                // Log update for debugging
                if (RobotBase.isSimulation()) {
                    System.out.println("Test pattern applied at " + Timer.getFPGATimestamp());
                }
            }
        };
    }
}
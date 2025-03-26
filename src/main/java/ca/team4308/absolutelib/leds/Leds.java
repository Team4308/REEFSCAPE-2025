package ca.team4308.absolutelib.leds;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Leds {
    private final int port;
    private final int length;
    private final boolean debug;
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private boolean isStarted = false;
    private LEDPattern currentPattern;
    private AddressableLEDBufferView fullBufferView;
    private AddressableLEDSim ledSim;
    public SimDevice m_simDevice;
    public SimDouble m_simR;
    public SimDouble m_simG;
    public SimDouble m_simB;
    public SimDouble m_simBrightness;
    @SuppressWarnings("unused")
    private long lastUpdateTimeMs = 0;
    /**
     * Creates a new LED strip controller.
     * @param port The PWM port the LED strip is connected to
     * @param length The number of LEDs in the strip
     * @param debug Enables debug messages Warning / Error Println statments are stil thrown if an error occurs 
     */
    public Leds(int port, int length, boolean debug) {
        this.port = port;
        this.length = length;
        this.ledStrip = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(length);
        this.ledStrip.setLength(length);
        this.ledStrip.setData(ledBuffer);
        this.debug = debug;
        this.fullBufferView = new AddressableLEDBufferView(ledBuffer, 0, length);

    }
    /**
     * Starts the LED output.
     * Must be called after configuration and before setting colors.
     */
    public void start() {
        if (!isStarted) {
            ledStrip.start();
            isStarted = true;
            
        }
    }

    /**
     * Stops the LED output.
     */
    public void stop() {
        if (isStarted) {
            ledStrip.stop();
            isStarted = false;
        }
    }

    public void setColor(int index, Color color)  {
        if (index >= 0 && index < length) {
            ledBuffer.setLED(index, color);
        }
    }

    /**
     * Sets the color of a specific LED.
     * @param index The LED index
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void setRGB(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, r, g, b);
        }
    }

    /**
     * Sets the HSV color of a specific LED.
     * @param index The LED index
     * @param h Hue (0-180)
     * @param s Saturation (0-255)
     * @param v Value (0-255) "Brightness" or "Value"
     */
    public void setHSV(int index, int h, int s, int v) {
        if (index >= 0 && index < length) {
            ledBuffer.setHSV(index, h, s, v);
        }
    }

    /**
     * Updates the LED strip with the current buffer contents.
     */
    public void update() {
        if (!isStarted) return;
        
        try {
            if (currentPattern != null) {
                try {
                    currentPattern.applyTo(fullBufferView);
                    if (debug) {SmartDashboard.putBoolean("LED/PatternApplied", true);}
                } catch (Exception e) {
                    
                   if (debug) { SmartDashboard.putString("LED/PatternError", e.getMessage()); }
                    for (int i = 0; i < length; i++) {
                        ledBuffer.setRGB(i, 255, 0, 0);
                        System.err.println("Error applying pattern: " + e.getMessage());
                    }
                }
            }
            
            ledStrip.setData(ledBuffer);
            if (length > 0) {
                Color c = ledBuffer.getLED(0);
                if (debug) {
                SmartDashboard.putNumber("LED/FirstPixelR", c.red);
                SmartDashboard.putNumber("LED/FirstPixelG", c.green);
                SmartDashboard.putNumber("LED/FirstPixelB", c.blue);
                }
            }
            
            if (RobotBase.isSimulation()) {   
                updateSimulation();
            }
        } catch (Exception e) {
            System.err.println("Error updating LEDs: " + e.getMessage());
            e.printStackTrace();
            SmartDashboard.putString("LED/ErrorMsg", e.getMessage());
        }
    }
    
    /**
     * Updates simulation values for the LEDs
     */
    private void updateSimulation() {
        double totalR = 0, totalG = 0, totalB = 0;
        int numLeds = this.ledBuffer.getLength();
        
        for (int i = 0; i < numLeds; i++) {
            Color color = this.ledBuffer.getLED(i);
            totalR += color.red;
            totalG += color.green;
            totalB += color.blue;
        }

        double avgLength = Math.max(1, numLeds);
        
        if (m_simDevice != null && m_simR != null && m_simG != null && 
            m_simB != null && m_simBrightness != null) {
            m_simR.set(totalR / avgLength);
            m_simG.set(totalG / avgLength);
            m_simB.set(totalB / avgLength);
            m_simBrightness.set(Math.max(Math.max(totalR, totalG), totalB) / avgLength);
        }
        if (debug) {
        SmartDashboard.putNumber("LED/Red", (totalR / avgLength));
        SmartDashboard.putNumber("LED/Green", (totalG / avgLength));
        SmartDashboard.putNumber("LED/Blue", (totalB / avgLength));
        }
    }

    /**
     * Updates the LED pattern if one is set.
     */
    public void updatePattern() {
        if (currentPattern != null) {
            currentPattern.applyTo(fullBufferView);
            update();
        }
    }

    /**
     * Gets the length of the LED strip.
     * @return The number of LEDs in the strip
     */
    public int getLength() {
        return length;
    }

    /**
     * Gets the port number the LED strip is connected to.
     * @return The PWM port number
     */
    public int getPort() {
        return port;
    }

    /**
     * Clears all LEDs by setting them to off (0,0,0).
     */
    public void clear() {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        update();
    }

    /**
     * Creates a view into a section of the LED strip.
     * @param startIndex The starting LED index
     * @param length The number of LEDs in the view
     * @return A new AddressableLEDBufferView
     */
    public AddressableLEDBufferView createBufferView(int startIndex, int length) {
        startIndex = Math.max(0, Math.min(startIndex, this.length - 1));
        length = Math.max(1, Math.min(length, this.length - startIndex));
        
        return new AddressableLEDBufferView(ledBuffer, startIndex, length);
    }

    /**
     * Creates a buffer view for a range of LEDs defined by start and end indices.
     * @param startIndex The starting LED index (inclusive)
     * @param endIndex The ending LED index (inclusive)
     * @return A new AddressableLEDBufferView
     */
    public AddressableLEDBufferView createRangeBufferView(int startIndex, int endIndex) {

        startIndex = Math.max(0, Math.min(startIndex, this.length - 1));
        endIndex = Math.max(startIndex, Math.min(endIndex, this.length - 1));
        
        int viewLength = endIndex - startIndex + 1;
        return new AddressableLEDBufferView(ledBuffer, startIndex, viewLength);
    }

    /**
     * Creates multiple equal-sized views of the LED strip.
     * @param numSections The number of sections to create
     * @return Array of AddressableLEDBufferView objects
     */
    public AddressableLEDBufferView[] splitIntoSections(int numSections) {
        if (numSections <= 0 || length % numSections != 0) {
            throw new IllegalArgumentException("Invalid number of sections");
        }

        int sectionLength = length / numSections;
        AddressableLEDBufferView[] views = new AddressableLEDBufferView[numSections];
        
        for (int i = 0; i < numSections; i++) {
            views[i] = createBufferView(i * sectionLength, sectionLength);
        }
        
        return views;
    }

    /**
     * Sets the pattern for the entire LED strip.
     * @param pattern The pattern to apply
     */
    public void setPattern(LEDPattern pattern) {
        @SuppressWarnings("unused")
        boolean isNewPattern = (this.currentPattern == null) || 
                              (pattern != null && this.currentPattern.getClass() != pattern.getClass());
        
        this.currentPattern = pattern;
        
        if (pattern != null && isStarted) {
            try {
                if (debug) {
                SmartDashboard.putBoolean("LED/PatternChanged", true);
                }
                if (pattern instanceof TimeBasedLEDPattern) {
                    ((TimeBasedLEDPattern) pattern).resetTimer();
                }
                pattern.applyTo(fullBufferView);
                ledStrip.setData(ledBuffer);
                
            } catch (Exception e) {
                throw new RuntimeException("Error applying pattern: " + e.getMessage());
            }
        }
    }

    /**
     * Sets the pattern for a specific range of LEDs.
     * @param pattern The pattern to apply
     * @param startIndex The starting LED index (inclusive)
     * @param endIndex The ending LED index (inclusive)
     */
    public void setPatternForRange(LEDPattern pattern, int startIndex, int endIndex) {
        if (pattern == null) return;
        startIndex = Math.max(0, Math.min(startIndex, length - 1));
        endIndex = Math.max(startIndex, Math.min(endIndex, length - 1));
        AddressableLEDBufferView rangeView = createBufferView(startIndex, endIndex - startIndex + 1);
        
        pattern.applyTo(rangeView);
        update();
    }

    /**
     * Applies a solid color to a specific buffer view.
     * @param view The buffer view to apply the color to
     * @param pattern The pattern to apply to the view
     */
    public void applyToView(AddressableLEDBufferView view, LEDPattern pattern) {
        pattern.applyTo(view);
    }
    
    /**
     * Applies a solid color to a specific buffer view.
     * @param view The buffer view to apply the color to
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void applyToView(AddressableLEDBufferView view, int r, int g, int b) {
        for (int i = 0; i < view.getLength(); i++) {
            view.setRGB(i, r, g, b);
        }
    }

    /**
     * Applies an HSV color to a specific buffer view.
     * @param view The buffer view to apply the color to
     * @param h Hue (0-180)
     * @param s Saturation (0-255)
     * @param v Value/Brightness (0-255)
     */
    public void applyHSVToView(AddressableLEDBufferView view, int h, int s, int v) {
        for (int i = 0; i < view.getLength(); i++) {
            view.setHSV(i, h, s, v);
        }
    }

    /**
     * Applies a custom operation to each LED in a buffer view.
     * @param view The buffer view to apply the operation to
     * @param operation The operation to apply to each LED index
     */
    public void applyOperationToView(AddressableLEDBufferView view, LEDOperation operation) {
        for (int i = 0; i < view.getLength(); i++) {
            operation.apply(view, i);
        }
    }

    /**
     * Functional interface for custom LED operations
     */
    @FunctionalInterface
    public interface LEDOperation {
        void apply(AddressableLEDBufferView view, int index);
    }

    /**
     * Applies a pattern to a specific buffer view.
     * @param view The buffer view to apply the pattern to
     * @param pattern The pattern to apply to the view
     */
    public void applyPattern(AddressableLEDBufferView view, LEDPattern pattern) {
        pattern.applyTo(view);
    }

    /**
     * Gets the current pattern.
     * @return The current pattern
     */
    public LEDPattern getPattern() {
        return currentPattern;
    }
    
    /**
     * Gets the underlying LED strip.
     * @return The AddressableLED object
     */
    public AddressableLED getAddressableLED() {
        return ledStrip;
    }
    
    /**
     * Checks if the LED strip is real (not simulated).
     * @return true if running on real hardware, false if simulated
     */
    public boolean isReal() {
        try {
            return !edu.wpi.first.wpilibj.RobotBase.isSimulation();
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Initialize the LED strip.
     * This is automatically called by the constructor.
     */
    public void initialize() {

    }

    /**
     * Gets the underlying LED buffer.
     * @return The AddressableLEDBuffer object
     */
    public AddressableLEDBuffer getBuffer() {
        return ledBuffer;
    }

    /**
     * Gets the AddressableLEDSim used for simulation.
     * @return The AddressableLEDSim object
     */
    public AddressableLEDSim getLEDSim() {
        return ledSim;
    }
    
    /**
     * Gets the raw LED data for simulation purposes.
     * This method is primarily used for simulation.
     * @return The LED buffer data
     */
    public byte[] getRawLEDData() {
        byte[] data = new byte[length * 4]; // No more then 4 bytes (rgbw)
        
        for (int i = 0; i < length; i++) {
            Color color = ledBuffer.getLED(i);
            int idx = i * 4;
            data[idx] = (byte)(color.red * 255);     // R
            data[idx + 1] = (byte)(color.green * 255); // G
            data[idx + 2] = (byte)(color.blue * 255);  // B
            data[idx + 3] = 0; // W (not used)
        }
        
        return data;
    }
}

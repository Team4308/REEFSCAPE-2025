package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents a view into an AddressableLEDBuffer.
 * Allows addressing a subset of LEDs in a strip as its own entity.
 */
public class AddressableLEDBufferView {
    private final AddressableLEDBuffer buffer;
    private final int startIndex;
    private final int length;

    /**
     * Creates a new AddressableLEDBufferView for a portion of an LED strip.
     * @param buffer The underlying LED buffer
     * @param startIndex The starting LED index
     * @param length The number of LEDs in the view
     */
    public AddressableLEDBufferView(AddressableLEDBuffer buffer, int startIndex, int length) {
        this.buffer = buffer;
        this.startIndex = Math.max(0, startIndex);
        this.length = Math.min(length, buffer.getLength() - this.startIndex);
    }

    /**
     * Sets the color of an LED in the view.
     * @param index The index within the view (0 is the first LED in the view)
     * @param color The color to set
     */
    public void setColor(int index, Color color) {
        if (index >= 0 && index < length) {
            buffer.setLED(startIndex + index, color);
        }
    }

    /**
     * Sets the RGB values of an LED in the view.
     * @param index The index within the view (0 is the first LED in the view)
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void setRGB(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            buffer.setRGB(startIndex + index, r, g, b);
        }
    }

    /**
     * Sets the HSV values of an LED in the view.
     * @param index The index within the view (0 is the first LED in the view)
     * @param h Hue (0-180)
     * @param s Saturation (0-255)
     * @param v Value/Brightness (0-255)
     */
    public void setHSV(int index, int h, int s, int v) {
        if (index >= 0 && index < length) {
            buffer.setHSV(startIndex + index, h, s, v);
        }
    }

    /**
     * Gets the color of an LED in the view.
     * @param index The index within the view (0 is the first LED in the view)
     * @return The color of the LED
     */
    public Color getLED(int index) {
        if (index >= 0 && index < length) {
            return buffer.getLED(startIndex + index);
        }
        return new Color(0, 0, 0);
    }

    /**
     * Gets the number of LEDs in the view.
     * @return The length of the view
     */
    public int getLength() {
        return length;
    }
}

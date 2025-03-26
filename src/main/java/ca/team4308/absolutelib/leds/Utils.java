package ca.team4308.absolutelib.leds;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Utility methods for LED patterns
 */
public class Utils {
    
    /**
     * Sets an LED color for an idle pattern segment
     * @param view The buffer view to apply to
     * @param ledIndex The LED index to set
     * @param position Position within the pattern
     * @param baseColor Base color for the pattern
     * @param patternLength Total pattern length
     * @param trailLength Length of the trailing effect
     */
    public static void setIdlePatternColor(
            AddressableLEDBufferView view, 
            int ledIndex, 
            int position, 
            Color baseColor, 
            int patternLength, 
            int trailLength) {
        
        // Default color is the base color at 10% brightness
        Color color = new Color(baseColor.red * 0.1, baseColor.green * 0.1, baseColor.blue * 0.1);
        
        // Check if in the trail segment
        if (position % patternLength < trailLength) {
            double intensity = 0.1 + 0.9 * (1.0 - ((double)(position % patternLength) / trailLength));
            color = new Color(
                baseColor.red * intensity, 
                baseColor.green * intensity, 
                baseColor.blue * intensity
            );
        }
        
        view.setColor(ledIndex, color);
    }

    /**
     * Calculates a fade value based on position in pattern.
     * @param position Current position in pattern
     * @param patternLength Total length of pattern
     * @param trailLength Length of fade trail
     * @return Fade value between 0.0 and 1.0
     */
    public static double getFadeValue(int position, int patternLength, int trailLength) {
        if (trailLength <= 0 || patternLength <= 0) {
            return 0.0;
        }

        if (position < trailLength) {
            return (double) position / trailLength;
        } else if (position >= patternLength - trailLength) {
            return (double) (patternLength - position) / trailLength;
        }
        return 1.0;
    }

    /**
     * Creates a breathing pattern that fades between a base color and white.
     * @param buffer LED buffer view to modify
     * @param position Current animation position
     * @param baseColor Base color to fade from
     * @param patternLength Length of the complete pattern
     * @param trailLength Length of the fade trail
     */
    public static void createBreathingPattern(AddressableLEDBufferView buffer, int position, 
            Color baseColor, int patternLength, int trailLength) {
        if (buffer == null || baseColor == null) {
            return;
        }

        for (int i = 0; i < buffer.getLength(); i++) {
            int adjustedPosition = position % (patternLength * 3);
            
            if (adjustedPosition < patternLength) {
                // First phase: fade up
                double fade = getFadeValue(adjustedPosition, patternLength, trailLength);
                double blendFactor = fade * 0.3;
                setBlendedColor(buffer, i, baseColor, blendFactor);
            } else if (adjustedPosition < patternLength * 2) {
                // Second phase: middle intensity
                double fade = getFadeValue(adjustedPosition - patternLength, patternLength, trailLength);
                double blendFactor = 0.3 + (fade * 0.4);
                setBlendedColor(buffer, i, baseColor, blendFactor);
            } else {
                // Third phase: fade to maximum
                double fade = getFadeValue(adjustedPosition - (patternLength * 2), patternLength, trailLength);
                double blendFactor = 0.7 + (fade * 0.3);
                setBlendedColor(buffer, i, baseColor, blendFactor);
            }
        }
    }

    /**
     * Sets a color blended between the base color and white.
     * @param buffer LED buffer to modify
     * @param index LED index
     * @param baseColor Base color to blend from
     * @param blendFactor Blend factor (0.0 = base color, 1.0 = white)
     */
    private static void setBlendedColor(AddressableLEDBufferView buffer, int index, Color baseColor, double blendFactor) {
        blendFactor = clamp(blendFactor, 0.0, 1.0);
        
        Color color = new Color(
            baseColor.red + (blendFactor * (1.0 - baseColor.red)),
            baseColor.green + (blendFactor * (1.0 - baseColor.green)),
            baseColor.blue + (blendFactor * (1.0 - baseColor.blue))
        );
        
        buffer.setColor(index, color);
    }

    /**
     * Clamps a value between a minimum and maximum.
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Blend colors safely with proper error checking
     */
    public static Color blendColors(Color c1, Color c2, double blend) {
        // Safety check for null colors
        if (c1 == null) return c2 != null ? c2 : new Color(0, 0, 0);
        if (c2 == null) return c1;
        
        blend = clamp(blend, 0.0, 1.0);
        
        double r = clamp(c1.red * (1.0 - blend) + c2.red * blend, 0.0, 1.0);
        double g = clamp(c1.green * (1.0 - blend) + c2.green * blend, 0.0, 1.0);
        double b = clamp(c1.blue * (1.0 - blend) + c2.blue * blend, 0.0, 1.0);
        
        return new Color(r, g, b);
    }
    
    /**
     * Scales a color's brightness
     * @param color The color to scale
     * @param scale The brightness scale factor
     * @return Scaled color
     */
    public static Color scaleColor(Color color, double scale) {
        scale = Math.max(0.0, scale);
        return new Color(
            Math.min(1.0, color.red * scale),
            Math.min(1.0, color.green * scale),
            Math.min(1.0, color.blue * scale)
        );
    }

        // Helper methods
    public static Color getAllianceColor() {
        try {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            // Fix: Properly handle the Optional to prevent null reference exceptions
            if (alliance.isPresent()) {
                return alliance.get() == Alliance.Blue ? 
                    new Color(0, 0, 1) : // Blue
                    new Color(1, 0, 0);  // Red
            }

            // Default to red if alliance can't be determined
            return new Color(1, 0, 0);
        } catch (Exception e) {
            // Fallback in case of any exception
            System.err.println("Error getting alliance color: " + e.getMessage());
            return new Color(1, 0, 0); // Default to red
        }
    }


        /**
     * Creates a breathing pattern that fades between colors
     */
    public static LEDPattern createBreathingPattern(Color baseColor, double periodSeconds) {
        return new TimeBasedLEDPattern() {
            private long startTime = System.currentTimeMillis();
            
            @Override
            public void resetTimer() {
                startTime = System.currentTimeMillis();
            }

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                double phase = (time % periodSeconds) / periodSeconds;
                double intensity = (Math.sin(phase * 2 * Math.PI) + 1) / 2;

                for (int i = 0; i < view.getLength(); i++) {
                    Color color = new Color(
                        baseColor.red * intensity,
                        baseColor.green * intensity,
                        baseColor.blue * intensity
                    );
                    view.setColor(i, color);
                }
            }
        };
    }


    /**
     * Creates an alliance-colored pattern (red or blue based on driver station)
     */
    public static LEDPattern getAlliancePattern() {
        Color color = getAllianceColor();
        return createSolidPattern(color);
    }

    /**
     * Creates a breathing alliance-colored pattern
     */
    public static LEDPattern getAllianceBreathing(double periodSeconds) {
        Color color = getAllianceColor();
        return createBreathingPattern(color, periodSeconds);
    }

    /**
     * Creates a solid color pattern
     */
    public static LEDPattern createSolidPattern(Color color) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, color);
                }
            }
        };
    }


}

package ca.team4308.absolutelib.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Random;


public class Patterns {
    public static final int DEFAULT_PATTERN_LENGTH = 5;
    public static final int DEFAULT_SCROLL_SPEED = 1;
    public static final double DEFAULT_BREATHE_PERIOD = 2000.0;
    public static final double DEFAULT_BLINK_PERIOD = 0.2;

    /**
     * Creates a standard idle pattern.
     * Gentle breathing effect with the default idle color.
     * @return LEDPattern for idle state
     */
    public static LEDPattern idle() {
        return Utils.createBreathingPattern(LEDConstants.DEFAULT_IDLE_COLOR, LEDConstants.getBreathePeriod());
    }

    /**
     * Creates a warning pattern.
     * Fast blinking with warning color.
     * @return LEDPattern for warning state
     */
    public static LEDPattern warning() {
        return createBlinkingPattern(LEDConstants.DEFAULT_WARNING_COLOR, LEDConstants.getBlinkPeriod());
    }

    /**
     * Creates an error pattern.
     * Rapid blinking with error color.
     * @return LEDPattern for error state
     */
    public static LEDPattern error() {
        return createBlinkingPattern(LEDConstants.DEFAULT_ERROR_COLOR, LEDConstants.getBlinkPeriod() * 0.5);
    }

    /**
     * Creates a success pattern.
     * Solid success color with optional pulse.
     * @return LEDPattern for success state
     */
    public static LEDPattern success() {
        return Utils.createSolidPattern(LEDConstants.DEFAULT_SUCCESS_COLOR);
    }

    /**
     * Creates a rainbow chase pattern.
     * Scrolling rainbow effect with customizable speed.
     * @return LEDPattern with rainbow chase effect
     */
    public static LEDPattern rainbowChase() {
        return createRainbowPattern(
            LEDConstants.DEFAULT_RAINBOW_SATURATION,
            LEDConstants.DEFAULT_RAINBOW_VALUE,
            LEDConstants.DEFAULT_RAINBOW_SPEED,
            1.0 / LEDConstants.getPatternLength()
        );
    }




    /**
     * Creates a rainbow pattern with customizable parameters
     */
    public static LEDPattern createRainbowPattern(int saturation, int value, double speedMPS, double spacing) {
        return new TimeBasedLEDPattern() {
            private long startTimeMs = System.currentTimeMillis();
            private boolean initialized = false;
            
            @Override
            public void resetTimer() {
                startTimeMs = System.currentTimeMillis();
                initialized = true;
            }
            
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                if (!initialized) {
                    startTimeMs = System.currentTimeMillis();
                    initialized = true;
                }
                
                double timeOffset = (System.currentTimeMillis() - startTimeMs) / 1000.0 * speedMPS * 180;
                
                int length = view.getLength();
                for (int i = 0; i < length; i++) {
                    // Calculate hue based on position and time
                    int hue = (int)((i * 180.0 / length * spacing + timeOffset) % 180);
                    view.setHSV(i, hue, saturation, value);
                }
            }
        };
    }


    /**
     * Creates a blinking pattern
     */
    public static LEDPattern createBlinkingPattern(Color color, double periodSeconds) {
        return new TimeBasedLEDPattern() {
            private long startTime = System.currentTimeMillis();
            
            @Override
            public void resetTimer() {
                startTime = System.currentTimeMillis();
            }

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                boolean isOn = ((time % periodSeconds) < (periodSeconds / 2));

                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, isOn ? color : new Color(0, 0, 0));
                }
            }
        };
    }

   
    /**
     * Creates a progress bar pattern
     */
    public static LEDPattern createProgressPattern(Color color, double progress) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int activeLength = (int)(view.getLength() * progress);
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, i < activeLength ? color : new Color(0, 0, 0));
                }
            }
        };
    }

    /**
     * Creates a chasing dot pattern.
     * Single color dot moving along the strip.
     * @param color The color of the moving dot
     * @return LEDPattern with chasing dot effect
     */
    public static LEDPattern chasingDot(Color color) {
        return new LEDPattern() {
            private long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                int position = (int)(time * LEDConstants.DEFAULT_CHASE_SPEED * view.getLength()) % view.getLength();
                
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, i == position ? color : new Color(0, 0, 0));
                }
            }
        };
    }

    /**
     * Creates a scrolling idle pattern that transitions from base color to white.
     * @param baseColor The base color to fade from
     * @param scrollSpeed The speed of scrolling
     * @return LEDPattern for scrolling idle state
     */
    public static LEDPattern scrollingIdle(Color baseColor, int scrollSpeed) {
        return new LEDPattern() {
            private int offset = 0;

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int patternLength = LEDConstants.getPatternLength();
                int trailLength = patternLength / 4; // Use 1/4 of pattern length for fade

                for (int i = 0; i < view.getLength(); i++) {
                    int position = (i + offset) % (patternLength * 3);
                    Utils.setIdlePatternColor(view, i, position, baseColor, patternLength, trailLength);
                }
                offset = (offset + scrollSpeed) % (patternLength * 3);
            }
        };
    }

    /**
     * Creates a default scrolling idle pattern using alliance colors.
     * @return LEDPattern for alliance-colored scrolling idle state
     */
    public static LEDPattern defaultScrollingIdle() {
        return scrollingIdle(Utils.getAllianceColor(), LEDConstants.getScrollSpeed());
    }

    /**
     * Creates the real AltF4 pattern - expanding dots that alternate team color/white
     * @return LEDPattern for the AltF4 effect that works reliably
     */
    public static LEDPattern altF4() {
        return new TimeBasedLEDPattern() {
            public long startTime = System.currentTimeMillis();
                        private final Random random = new Random();
                        private int expansionCenter = -1;
                        private double expansionSize = 0;
                        private boolean useTeamColor = true;
                        
                        @Override
                        public void resetTimer() {
                            startTime = System.currentTimeMillis();
                expansionCenter = -1; // Force recalculation
            }
            
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                // Calculate time values
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                double cycleTime = time % 3.0; // 3 second cycle
                
                // Get colors
                Color teamColor = Utils.getAllianceColor();
                Color white = new Color(1, 1, 1);
                
                // Initialize or restart expansion when needed
                if (expansionCenter < 0 || cycleTime < 0.1) {
                    expansionCenter = random.nextInt(view.getLength());
                    expansionSize = 1;
                    useTeamColor = !useTeamColor; // Alternate colors each cycle
                }
                
                // Determine fill and dot colors
                Color fillColor = useTeamColor ? white : teamColor;
                Color dotColor = useTeamColor ? teamColor : white;
                
                // Fill entire strip with base color
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, fillColor);
                }
                
                // Calculate expansion size (grows from 1 to half the strip)
                expansionSize = Math.min(view.getLength() / 2.0, expansionSize + 0.3);
                
                // Apply the expanding dot to the LEDs
                int radius = (int)Math.ceil(expansionSize);
                for (int i = -radius; i <= radius; i++) {
                    // Calculate LED position with wraparound
                    int pos = (expansionCenter + i) % view.getLength();
                    if (pos < 0) pos += view.getLength();
                    
                    // Calculate fade based on distance from center
                    double distance = Math.abs(i);
                    if (distance > expansionSize) continue;
                    
                    // Calculate blend factor (stronger in center, fades at edges)
                    double blend = 1.0 - (distance / expansionSize);
                    blend = Math.pow(blend, 2); // Square for smoother falloff
                    
                    // Apply blended color
                    view.setColor(pos, Utils.blendColors(fillColor, dotColor, blend));
                }
                
                // Log debug info
                SmartDashboard.putNumber("AltF4/Center", expansionCenter);
                SmartDashboard.putNumber("AltF4/Size", expansionSize);
                SmartDashboard.putBoolean("AltF4/UseTeamColor", useTeamColor);
            }
        };
    }
    
    /**
     * Creates a TeamFlag pattern that works reliably - scrolling stripes with team color
     * @return LEDPattern for team flag effect
     */
    public static LEDPattern teamFlag() {
        return new TimeBasedLEDPattern() {
            private long startTime = System.currentTimeMillis();
                        
                        @Override
                        public void resetTimer() {
                            startTime = System.currentTimeMillis();
            }
            
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                // Get colors
                Color teamColor = Utils.getAllianceColor();
                Color white = new Color(1, 1, 1);
                
                // Calculate scrolling offset
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                int offset = (int)(time * 10) % (view.getLength() * 2); // 10 pixels per second
                
                // Create flag segments (divide strip into 6 segments)
                int segmentLength = Math.max(1, view.getLength() / 6);
                
                for (int i = 0; i < view.getLength(); i++) {
                    // Determine which segment this pixel is in
                    int adjustedPos = (i + offset) % view.getLength();
                    int segment = adjustedPos / segmentLength;
                    
                    // For even segments use team color, for odd use white
                    if (segment % 2 == 0) {
                        view.setColor(i, teamColor);
                    } else {
                        view.setColor(i, white);
                    }
                }
                
                // Log debug info
                SmartDashboard.putNumber("TeamFlag/Time", time);
                SmartDashboard.putNumber("TeamFlag/Offset", offset);
                SmartDashboard.putNumber("TeamFlag/SegmentLength", segmentLength);
            }
        };
    }
    
    /**
     * Creates a TeamFlag pattern with fading between segments
     * @return LEDPattern for team flag effect with fading
     */
    public static LEDPattern teamFlagWithFade() {
        return new TimeBasedLEDPattern() {
            private long startTime = System.currentTimeMillis();
            
            @Override
            public void resetTimer() {
                startTime = System.currentTimeMillis();
            }
            
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                // Get colors
                Color teamColor = Utils.getAllianceColor();
                Color white = new Color(1, 1, 1);
                
                // Calculate scrolling offset (moves at 5 pixels per second)
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                double pixelsPerSecond = 5.0;
                double position = (time * pixelsPerSecond) % view.getLength();
                
                // Divide into 4 segments
                int segmentLength = Math.max(1, view.getLength() / 4);
                
                for (int i = 0; i < view.getLength(); i++) {
                    // Calculate position in the scrolling pattern
                    int adjustedPos = (i + (int)position) % view.getLength();
                    
                    // Determine segment and position within segment (0.0 to 1.0)
                    int segment = adjustedPos / segmentLength;
                    double segmentPos = (adjustedPos % segmentLength) / (double)segmentLength;
                    
                    // Apply color based on segment with fading
                    if (segment % 2 == 0) {
                        // Fade from team color to white
                        view.setColor(i, Utils.blendColors(teamColor, white, segmentPos));
                    } else {
                        // Fade from white to team color
                        view.setColor(i, Utils.blendColors(white, teamColor, segmentPos));
                    }
                }
                
                // Log debug info
                SmartDashboard.putNumber("TeamFlag/SegmentLength", segmentLength);
                SmartDashboard.putNumber("TeamFlag/Position", position);
            }
        };
    }

    /**
     * Creates a solid pattern that alternates team colors
     * @return LEDPattern that alternates team colors
     */
    public static LEDPattern teamPulse() {
        return new LEDPattern() {
            private long startTime = System.currentTimeMillis();
            
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                Color teamColor = Utils.getAllianceColor();
                Color altColor = new Color(1, 1, 1); // White
                
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                double phase = (Math.sin(time * Math.PI) + 1) / 2;
                
                Color blendedColor = Utils.blendColors(teamColor, altColor, phase);
                
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, blendedColor);
                }
            }
        };
    }


}

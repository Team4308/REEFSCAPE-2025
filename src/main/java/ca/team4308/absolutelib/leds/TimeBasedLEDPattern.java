package ca.team4308.absolutelib.leds;

/**
 * Interface for LED patterns that are time-based and need
 * their timer reset when reapplied
 */
public interface TimeBasedLEDPattern extends LEDPattern {
    /**
     * Resets the internal timer of the pattern
     * This ensures animations restart from the beginning when patterns change
     */
    void resetTimer();
}

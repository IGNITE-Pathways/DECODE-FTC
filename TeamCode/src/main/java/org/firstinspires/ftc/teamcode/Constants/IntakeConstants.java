package org.firstinspires.ftc.teamcode.Constants;

/**
 * Constants for the Intake component.
 * Contains power values, ramp rates, thresholds, and safety settings.
 */
public class IntakeConstants {
    // Private constructor to prevent instantiation
    private IntakeConstants() {
        throw new IllegalStateException("Utility class");
    }
    
    // Power constants
    public static final double DEFAULT_FORWARD_POWER = 0.6;
    public static final double DEFAULT_REVERSE_POWER = -0.6;
    public static final double SLOW_FORWARD_POWER = 0.3;
    public static final double SLOW_REVERSE_POWER = -0.3;
    
    // Power ramping
    public static final double DEFAULT_RAMP_RATE = 0.05; // Power change per update
    
    // Thresholds
    public static final double POWER_THRESHOLD = 0.01; // Minimum power to be considered "running"
    
    // Safety
    public static final long MAX_RUNTIME_MS = 30000; // 30 second safety timeout
}

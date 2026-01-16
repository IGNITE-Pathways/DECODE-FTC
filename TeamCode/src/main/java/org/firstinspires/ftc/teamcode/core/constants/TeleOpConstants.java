package org.firstinspires.ftc.teamcode.core.constants;

/**
 * Centralized TeleOp configuration constants.
 * All teleop-related settings in one place for easy tuning.
 */
public class TeleOpConstants {

    private TeleOpConstants() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== FLYWHEEL ====================
    public static final double FLYWHEEL_DEFAULT_POWER = 0.0;  // Start at 0%
    public static final double FLYWHEEL_POWER_INCREMENT = 0.1; // 10% per dpad press
    public static final double FLYWHEEL_AUTO_SHOOT_POWER = 0.6; // Power when auto-shooting

    // ==================== HOOD ====================
    public static final double HOOD_DEFAULT_POSITION = 0.75;
    public static final double HOOD_INCREMENT = 0.2; // Changed from 0.05 to 0.2
    public static final double HOOD_MIN = 0.5;
    public static final double HOOD_MAX = 0.9;

    // ==================== TRANSFER RAMP ====================
    public static final double TRANSFER_UP_POSITION = 0.65;
    public static final double TRANSFER_DOWN_POSITION = 0.5;

    // ==================== INTAKE ====================
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER = -0.7;

    // ==================== TURRET ====================
    public static final double TURRET_LOCKED_POSITION = 0.0;  // Manual locked position
    public static final boolean TURRET_AUTO_LOCK_ENABLED = true; // Enable auto-lock by default

    // ==================== DRIVE TRAIN ====================
    // Speed multipliers for different modes
    public static final double DRIVE_SLOW_SPEED = 0.35;
    public static final double DRIVE_NORMAL_SPEED = 0.7;
    public static final double DRIVE_TURBO_SPEED = 1.0;

    // ==================== AUTO SHOOT TIMING ====================
    // Delays for automated shooting sequence
    public static final long AUTO_SHOOT_RAMP_UP_DELAY_MS = 500;  // Wait for flywheel to spin up
    public static final long AUTO_SHOOT_INTAKE_DELAY_MS = 1000;  // How long to run intake
}

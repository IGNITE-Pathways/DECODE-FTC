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
    public static final double FLYWHEEL_AUTO_SHOOT_POWER = 0.70; // Power when auto-shooting

    // ==================== HOOD ====================
    public static final double HOOD_DEFAULT_POSITION = 0.4;
    public static final double HOOD_INCREMENT = 0.2; // Changed from 0.05 to 0.2
    public static final double HOOD_MIN = 0.5;
    public static final double HOOD_MAX = 0.9;

    // ==================== DISTANCE PRESETS ====================
    // 2 feet preset
    public static final double PRESET_2FT_POWER = 0.68;
    public static final double PRESET_2FT_HOOD = 0.32;
    // 6 feet preset
    public static final double PRESET_6FT_POWER = 0.73;
    public static final double PRESET_6FT_HOOD = 0.32;
    // 10 feet preset
    public static final double PRESET_10FT_POWER = 0.78;
    public static final double PRESET_10FT_HOOD = 0.70;

    // ==================== AUTO SHOOT SEQUENCE TIMING ====================
    public static final long SPIN_UP_TIME_MS = 1500;        // Flywheel spin-up before feeding
    public static final long FEED_DURATION_MS = 500;        // How long to feed each ball
    public static final long PAUSE_DURATION_MS = 700;       // Pause between balls for flywheel recovery

    // ==================== TRANSFER RAMP ====================
    public static final double TRANSFER_UP_POSITION = 0.67;
    public static final double TRANSFER_DOWN_POSITION = 0.5;

    // ==================== INTAKE ====================
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER = -0.7;

    // ==================== TURRET ====================
    public static final double TURRET_LOCKED_POSITION = 0.1;  // Manual locked position
    public static final boolean TURRET_AUTO_LOCK_ENABLED = true; // Enable auto-lock by default


    // ==================== AUTO SHOOT TIMING ====================
    // Delays for automated shooting sequence (Y button only)
    public static final long AUTO_SHOOT_FLYWHEEL_TO_RAMP_MS = 1200;  // Flywheel starts, then 1.2s later ramp goes up
    public static final long AUTO_SHOOT_RAMP_TO_INTAKE_MS = 300;     // Ramp up, then 0.5s later intake starts
    public static final long AUTO_SHOOT_INTAKE_DURATION_MS = 5000;   // How long to run intake
    public static final long AUTO_SHOOT_RAMP_HOLD_DELAY_MS = 2500;   // Keep ramp up after feeding

    // ==================== DRIVER FEEL & COMFORT ====================
    // Non-linear joystick curves (higher = more precision at low speeds)
    public static final double JOYSTICK_CURVE_EXPONENT = 2.5;  // 1.0 = linear, 2.0 = squared, 2.5 = smoother

    // Acceleration limiting (units per loop)
    public static final double MAX_ACCELERATION = 0.08;  // Prevents jerky movements

    // ==================== GAMEPAD RUMBLE PATTERNS ====================
    // Rumble durations in milliseconds
    public static final int RUMBLE_SHORT = 100;      // Quick pulse
    public static final int RUMBLE_MEDIUM = 300;     // Medium pulse
    public static final int RUMBLE_LONG = 500;       // Long pulse
    public static final int RUMBLE_DOUBLE = 150;     // Double pulse timing

    // ==================== ONE-BUTTON MACROS ====================
    // Ready to shoot macro settings
    public static final double MACRO_SHOOT_READY_POWER = 0.7;
    public static final double MACRO_SHOOT_READY_HOOD = 0.4;

    // ==================== PERFORMANCE & BATTERY OPTIMIZATION ====================
    // Joystick dead zone - ignore tiny stick movements
    public static final double JOYSTICK_DEAD_ZONE = 0.05;  // Ignore inputs below 5%

    // Flywheel auto-shutoff - prevent battery waste
    public static final long FLYWHEEL_AUTO_SHUTOFF_MS = 10000;  // 10 seconds of no shooting
}

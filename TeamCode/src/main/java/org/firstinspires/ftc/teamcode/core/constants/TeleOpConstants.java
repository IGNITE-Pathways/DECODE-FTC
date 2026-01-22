package org.firstinspires.ftc.teamcode.core.constants;

/**
 * Centralized TeleOp configuration constants.
 * All teleop-related settings in one place for easy tuning.
 */
public class TeleOpConstants {

    private TeleOpConstants() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== DEFAULT SHOOTER SETTINGS ====================
    public static final double DEFAULT_FLYWHEEL_POWER = 0.80;
    public static final double DEFAULT_HOOD_POSITION = 0.60;

    // ==================== DRIVE SPEED SETTINGS ====================
    public static final double SPEED_SHOOTING_MULTIPLIER = 0.85;  // Reduce speed while flywheel on

    // ==================== LIMELIGHT DISTANCE CALCULATION ====================
    public static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032;  // 8 inches
    public static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;
    public static final int IMAGE_WIDTH_PIXELS = 1280;
    public static final int IMAGE_HEIGHT_PIXELS = 720;

    // ==================== LIMELIGHT DISTANCE PRESETS ====================
    // Close range
    public static final double DISTANCE_CLOSE_MIN = 2.37;
    public static final double DISTANCE_CLOSE_MAX = 4.50;
    public static final double CLOSE_FLYWHEEL_POWER = 0.70;
    public static final double CLOSE_HOOD_POSITION = 0.60;

    // Mid range
    public static final double DISTANCE_MID_MIN = 4.50;
    public static final double DISTANCE_MID_MAX = 6.20;
    public static final double MID_FLYWHEEL_POWER = 0.85;
    public static final double MID_HOOD_POSITION = 0.70;

    // Far range
    public static final double DISTANCE_FAR_MIN = 6.20;
    public static final double DISTANCE_FAR_MAX = 8.00;
    public static final double FAR_FLYWHEEL_POWER = 0.85;
    public static final double FAR_HOOD_POSITION = 0.75;

    // ==================== INPUT SETTINGS ====================
    public static final double TRIGGER_DEADZONE = 0.1;
    public static final double JOYSTICK_DEAD_ZONE = 0.05;

    // ==================== TURRET ====================
    public static final double TURRET_LOCKED_POSITION = 0.45;
    public static final boolean TURRET_AUTO_LOCK_ENABLED = true;

    // ==================== TRANSFER RAMP ====================
    public static final double TRANSFER_UP_POSITION = 0.67;
    public static final double TRANSFER_DOWN_POSITION = 0.5;

    // ==================== INTAKE ====================
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER = -0.7;

    // ==================== AUTO SHOOT SEQUENCE TIMING ====================
    public static final long SPIN_UP_TIME_MS = 1250;
    public static final long FEED_DURATION_MS = 500;
    public static final long PAUSE_DURATION_MS = 700;

    // ==================== GAMEPAD RUMBLE PATTERNS ====================
    public static final int RUMBLE_SHORT = 100;
    public static final int RUMBLE_MEDIUM = 300;
    public static final int RUMBLE_LONG = 500;
    public static final int RUMBLE_DOUBLE = 150;

    // ==================== PERFORMANCE ====================
    public static final long FLYWHEEL_AUTO_SHUTOFF_MS = 10000;
}

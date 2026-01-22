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
    public static final double DEFAULT_FLYWHEEL_POWER = 0.80;  // 80% default
    public static final double DEFAULT_HOOD_POSITION = 0.25;   // Lowest position default

    // ==================== DRIVE SPEED SETTINGS ====================
    public static final double SPEED_SHOOTING_MULTIPLIER = 0.85;  // Reduce speed while flywheel on

    // ==================== LIMELIGHT DISTANCE CALCULATION ====================
    public static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032;  // 8 inches
    public static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;
    public static final int IMAGE_WIDTH_PIXELS = 1280;
    public static final int IMAGE_HEIGHT_PIXELS = 720;

    // ==================== LIMELIGHT DISTANCE PRESETS (TUNED RANGES) ====================
    // Range 1: 2.47 - 2.84 ft
    public static final double RANGE_1_MIN = 2.47;
    public static final double RANGE_1_MAX = 2.84;
    public static final double RANGE_1_FLYWHEEL_POWER = 0.70;
    public static final double RANGE_1_HOOD_POSITION = 0.40;

    // Range 2: 2.84 - 3.2 ft
    public static final double RANGE_2_MIN = 2.84;
    public static final double RANGE_2_MAX = 3.20;
    public static final double RANGE_2_FLYWHEEL_POWER = 0.70;
    public static final double RANGE_2_HOOD_POSITION = 0.55;

    // Range 3: 3.21 - 4.0 ft
    public static final double RANGE_3_MIN = 3.21;
    public static final double RANGE_3_MAX = 4.00;
    public static final double RANGE_3_FLYWHEEL_POWER = 0.75;
    public static final double RANGE_3_HOOD_POSITION = 0.50;

    // Range 4: 4.0 - 4.5 ft
    public static final double RANGE_4_MIN = 4.00;
    public static final double RANGE_4_MAX = 4.50;
    public static final double RANGE_4_FLYWHEEL_POWER = 0.80;
    public static final double RANGE_4_HOOD_POSITION = 0.50;

    // Range 5: 4.6 - 5.0 ft
    public static final double RANGE_5_MIN = 4.60;
    public static final double RANGE_5_MAX = 5.00;
    public static final double RANGE_5_FLYWHEEL_POWER = 0.80;
    public static final double RANGE_5_HOOD_POSITION = 0.70;

    // Range 6: 4.84 - 5.25 ft
    public static final double RANGE_6_MIN = 4.84;
    public static final double RANGE_6_MAX = 5.25;
    public static final double RANGE_6_FLYWHEEL_POWER = 0.85;
    public static final double RANGE_6_HOOD_POSITION = 0.65;

    // Range 7: 5.2 - 5.7 ft
    public static final double RANGE_7_MIN = 5.20;
    public static final double RANGE_7_MAX = 5.70;
    public static final double RANGE_7_FLYWHEEL_POWER = 0.85;
    public static final double RANGE_7_HOOD_POSITION = 0.65;

    // Far shooting zone (beyond 5.7 ft)
    public static final double RANGE_FAR_MIN = 5.70;
    public static final double RANGE_FAR_FLYWHEEL_POWER = 1.00;  // 100% max power
    public static final double RANGE_FAR_HOOD_POSITION = 0.67;   // 0.65-0.68 range, using middle

    // ==================== INPUT SETTINGS ====================
    public static final double TRIGGER_DEADZONE = 0.1;
    public static final double JOYSTICK_DEAD_ZONE = 0.05;

    // ==================== TURRET ====================
    public static final double TURRET_LOCKED_POSITION = 0.2;
    public static final boolean TURRET_AUTO_LOCK_ENABLED = true;

    // ==================== TRANSFER RAMP ====================
    public static final double TRANSFER_UP_POSITION = 0.75;  // Raised higher for better feeding
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

package org.firstinspires.ftc.teamcode.Constants;

/**
 * Centralized turret configuration constants.
 * All turret tuning values in one place for easy adjustment.
 */
public class TurretConstants {

    private TurretConstants() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== APRILTAG IDS ====================
    public static final int BLUE_ALLIANCE_TAG = 20;
    public static final int RED_ALLIANCE_TAG = 24;
    public static final int OBELISK_TAG_1 = 21;
    public static final int OBELISK_TAG_2 = 22;
    public static final int OBELISK_TAG_3 = 23;

    // ==================== SERVO LIMITS ====================
    public static final double SERVO_MIN = 0.375;
    public static final double SERVO_MAX = 0.8;
    public static final double SERVO_CENTER = 0.5;

    // ==================== PID GAINS (Turret.java) ====================
    public static final double KP = 0.0004;
    public static final double KI = 0.0;
    public static final double KD = 0.0004;
    public static final double MAX_OUTPUT = 0.0007;
    public static final double MAX_INTEGRAL = 50.0;

    // ==================== ALIGNMENT TOLERANCES ====================
    public static final double TARGET_TX = 0.0;           // Center target
    public static final double IDEAL_RANGE = 3.0;         // Degrees - alignment tolerance
    public static final double NEAR_RANGE_THRESHOLD = 1.5; // Multiplier for "near" detection

    // ==================== SCANNING BEHAVIOR ====================
    public static final double SCAN_SPEED = 0.003;
    public static final double SEARCH_RANGE = 0.1;

    // ==================== OSCILLATION DAMPING ====================
    public static final double OSCILLATION_DAMPING_RATE = 0.5;  // 50% reduction per cycle
    public static final double MIN_DAMPING = 0.05;              // Minimum 5% speed
    public static final int MAX_OSCILLATION_COUNT = 20;
    public static final double MAX_RANGE_EXPANSION = 3.0;       // Max 3x range expansion

    // ==================== TURRETLOCK OPTIMIZED GAINS ====================
    // These are more conservative for stability
    public static final double OPTIMIZED_LATENCY_COMPENSATION = 0.3;
    public static final double OPTIMIZED_IMU_FEEDFORWARD = -0.0005;
    public static final double OPTIMIZED_MAX_IMU_OUTPUT = 0.004;
    public static final double OPTIMIZED_IMU_SMOOTHING = 0.15;

    public static final double OPTIMIZED_TRIM_KP = 0.0006;
    public static final double OPTIMIZED_TRIM_KI = 0.000005;
    public static final double OPTIMIZED_TRIM_KD = 0.001;
    public static final double OPTIMIZED_MAX_TRIM = 0.003;

    public static final double OPTIMIZED_ACQUIRE_KP = 0.0012;
    public static final double OPTIMIZED_ACQUIRE_MAX = 0.012;

    public static final double OPTIMIZED_TRACK_DEADBAND = 3.0;
    public static final double OPTIMIZED_HOLD_DEADBAND = 4.0;
    public static final double OPTIMIZED_HOLD_MAX_OUTPUT = 0.002;

    // ==================== CAMERA CONSTANTS (Distance Calculation) ====================
    public static final double APRILTAG_HEIGHT_METERS = 0.2032;
    public static final double CAMERA_VFOV_DEGREES = 49.5;
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;
}

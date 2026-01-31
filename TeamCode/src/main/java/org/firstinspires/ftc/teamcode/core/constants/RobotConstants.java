package org.firstinspires.ftc.teamcode.core.constants;

/**
 * *** MASTER ROBOT CONSTANTS - SINGLE SOURCE OF TRUTH ***
 * ALL competition settings are here: shooter, drive, turret, limelight, etc.
 *
 * AFTER TUNING PIDF:
 * 1. Run FlywheelPIDFTuner, press B to save gains to telemetry log
 * 2. Copy the values from telemetry log
 * 3. Update the PIDF constants below (lines 20-24)
 * 4. Recompile and test in teleop
 *
 * AFTER TESTING SHOOTING RANGES:
 * 1. Shoot from each distance, note if shots fall short/overshoot
 * 2. Update RPM values for each range (lines 59, 65, 71, 77, 83, 89, 95, 100)
 * 3. Recompile and test
 */
public class RobotConstants {

    private RobotConstants() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== PIDF VELOCITY CONTROL (TUNE THESE!) ====================
    // Use FlywheelPIDFTuner to find optimal values, then update here
    // These values are used by BOTH the tuner and competition teleop

    public static final boolean USE_VELOCITY_CONTROL = true;  // Set false to use power mode (fallback)
    public static final double DEFAULT_TARGET_RPM = 2400;     // Default flywheel speed (Range 1 close shot)

    // PIDF Gains - TUNE USING FlywheelPIDFTuner!
    // Last tuned: These values provide good performance across 2400-3500 RPM range
    public static final double FLYWHEEL_KP = 0.023550;  // Proportional gain
    public static final double FLYWHEEL_KI = 0.000025;  // Integral gain
    public static final double FLYWHEEL_KD = 0.000325;  // Derivative gain
    public static final double FLYWHEEL_KF = 0.000235;  // Feedforward gain

    // ==================== FLYWHEEL STARTUP BOOST (Faster Spinup) ====================
    // These settings control the initial power burst when the flywheel starts spinning
    // Adjust these if spinup is too slow or too aggressive
    public static final double SPINUP_BOOST_DURATION = 0.5;     // Duration of boost in seconds
    public static final double SPINUP_BOOST_POWER = 0.95;       // Power during boost (0.0-1.0)
    public static final double SPINUP_RPM_THRESHOLD = 0.8;      // Disable boost when at this % of target RPM

    // ==================== FLYWHEEL POWER (Legacy - only used if velocity control OFF) ====================
    public static final double FLYWHEEL_DEFAULT_POWER = 0.6;
    public static final double FLYWHEEL_SHOOTING_POWER = 0.8;
    public static final double FLYWHEEL_LOW_POWER = 0.5;
    public static final double FLYWHEEL_MAX_POWER = 1.0;

    // ==================== HOOD POSITIONS ====================
    public static final double HOOD_DEFAULT_POSITION = 0.55;  // Default (Range 1 close shot)
    public static final double HOOD_MIN_POSITION = 0.5;      // Lowest angle (far shots)
    public static final double HOOD_MAX_POSITION = 0.9;      // Highest angle (close shots)
    public static final double HOOD_INCREMENT = 0.05;

    // ==================== INTAKE POWER ====================
    public static final double INTAKE_DEFAULT_POWER = -1;
    public static final double INTAKE_SHOOTING_POWER = 1.0;

    // ==================== TRANSFER POSITIONS ====================
    public static final double TRANSFER_UP_POSITION = 0.67;  // Max height - servo limit
    public static final double TRANSFER_DOWN_POSITION = 0.5;

    // ==================== LIMELIGHT DISTANCE-BASED SHOOTING PRESETS ====================
    // After testing at each distance, adjust these RPM values for accurate shooting
    // Range format: MIN distance, MAX distance, Target RPM, Hood position

    // Range 1: 2.45 - 3.45 ft (Close range)
    public static final double RANGE_1_MIN = 2.45;
    public static final double RANGE_1_MAX = 3.45;
    public static final double RANGE_1_FLYWHEEL_RPM = 2400;
    public static final double RANGE_1_HOOD_POSITION = 0.55;

    // Range 2: 3.46 - 4.65 ft (Mid-close range)
    public static final double RANGE_2_MIN = 3.46;
    public static final double RANGE_2_MAX = 4.65;
    public static final double RANGE_2_FLYWHEEL_RPM = 2600;
    public static final double RANGE_2_HOOD_POSITION = 0.65;

    // Range 3: 4.66 - 5.35 ft (Mid range)
    public static final double RANGE_3_MIN = 4.66;
    public static final double RANGE_3_MAX = 5.35;
    public static final double RANGE_3_FLYWHEEL_RPM = 2700;
    public static final double RANGE_3_HOOD_POSITION = 0.70;

    // Range 4: 5.36 - 6.00 ft (Mid-far range)
    public static final double RANGE_4_MIN = 5.36;
    public static final double RANGE_4_MAX = 6.00;
    public static final double RANGE_4_FLYWHEEL_RPM = 2800;
    public static final double RANGE_4_HOOD_POSITION = 0.70;

    // Range 5: 6.01 - 7.00 ft (Far range start) - NEW!
    public static final double RANGE_5_MIN = 6.01;
    public static final double RANGE_5_MAX = 7.00;
    public static final double RANGE_5_FLYWHEEL_RPM = 2950;  // Tune this!
    public static final double RANGE_5_HOOD_POSITION = 0.72;

    // Range 6: 7.01 - 8.50 ft (Far range mid) - NEW!
    public static final double RANGE_6_MIN = 7.01;
    public static final double RANGE_6_MAX = 8.50;
    public static final double RANGE_6_FLYWHEEL_RPM = 3150;  // Tune this!
    public static final double RANGE_6_HOOD_POSITION = 0.73;

    // Range 7: 8.51 - 10.00 ft (Far range end) - NEW!
    public static final double RANGE_7_MIN = 8.51;
    public static final double RANGE_7_MAX = 10.00;
    public static final double RANGE_7_FLYWHEEL_RPM = 3300;  // Tune this!
    public static final double RANGE_7_HOOD_POSITION = 0.74;

    // Far shooting zone (10+ ft) - Maximum distance on field (back wall cycling)
    public static final double RANGE_FAR_MIN = 10.0;
    public static final double RANGE_FAR_FLYWHEEL_RPM = 3450;  // Increased by 50 for back wall cycling
    public static final double RANGE_FAR_HOOD_POSITION = 0.735;  // Lowered by 0.035 for flatter trajectory

    // ==================== LIMELIGHT CAMERA SPECS ====================
    public static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032;  // 8 inches
    public static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;
    public static final int IMAGE_WIDTH_PIXELS = 1280;
    public static final int IMAGE_HEIGHT_PIXELS = 720;

    // ==================== DRIVE CONTROL SETTINGS ====================
    public static final double SPEED_SHOOTING_MULTIPLIER = 0.85;  // Reduce speed while flywheel on
    public static final double JOYSTICK_DEAD_ZONE = 0.05;
    public static final double TRIGGER_DEADZONE = 0.1;

    // Driver comfort tuning (adjust these to fix driving feel)
    public static final double DRIVE_INPUT_CURVE = 1.0;      // 1.0=linear (100% speed), 1.5=slight curve, 2.0=squared
    public static final double ROTATION_SENSITIVITY = 1.0;   // 1.0=full speed, 0.8=slower rotation
    public static final boolean ENABLE_BATTERY_COMPENSATION = false;  // Try false if driving feels inconsistent

    // ==================== TURRET MANUAL CONTROL ====================
    public static final double TURRET_MANUAL_INCREMENT = 0.01;          // DPAD increment (reduced for finer control)
    public static final double TURRET_MANUAL_STICK_DEADZONE = 0.1;     // Stick deadzone
    public static final double TURRET_MANUAL_STICK_SENSITIVITY = 0.002; // Stick scaling (reduced for much finer control)
    public static final double TURRET_MIN_POSITION = 0.1;
    public static final double TURRET_MAX_POSITION = 0.9;
    public static final double TURRET_CENTER_POSITION = 0.5;

    // ==================== TURRET AUTO-TRACKING PID ====================
    // Adjust these if turret oscillates or is too slow to lock
    // NOTE: Turret uses CONTINUOUS TRACKING - it never stops moving to follow the AprilTag
    // Values match ActualTurretLockOn (v2) for optimal performance

    public static final double TURRET_KP = 0.008;                // Proportional gain - increase for faster, decrease if oscillates
    public static final double TURRET_KD = 0.005;                // Derivative gain (damping) - increase to reduce overshoot
    public static final double TURRET_POSITION_TOLERANCE = 2.5;  // degrees - "locked" status threshold (wider to prevent hunting)
    public static final double TURRET_TX_FILTER_ALPHA = 0.3;     // Low-pass filter: 0.1=smooth/slow, 0.5=fast/jittery

    // ==================== DISTANCE-BASED HOOD LOOKUP TABLE ====================
    // Distance in feet -> Hood position
    // Closer = higher hood position (steeper angle)
    // Farther = lower hood position (flatter angle)
    //
    // These values need to be calibrated on the actual robot:
    // 1. Set robot at known distances (2ft, 4ft, 6ft, 8ft, 10ft)
    // 2. Adjust hood until shots consistently score
    // 3. Record the optimal hood position for each distance

    // Distance breakpoints (in feet)
    public static final double[] DISTANCE_BREAKPOINTS = {
        2.0,   // Very close
        4.0,   // Close
        6.0,   // Medium
        8.0,   // Far
        10.0,  // Very far
        12.0   // Maximum range
    };

    // Hood positions for each distance breakpoint
    // Higher value = steeper angle (for close shots)
    // Lower value = flatter angle (for far shots)
    public static final double[] HOOD_POSITIONS = {
        0.85,  // 2ft  - steep angle for close
        0.80,  // 4ft  - slightly lower
        0.75,  // 6ft  - medium angle
        0.70,  // 8ft  - flatter
        0.65,  // 10ft - even flatter
        0.60   // 12ft - flat for max distance
    };

    /**
     * Get the optimal hood position based on distance to target.
     * Uses linear interpolation between calibrated breakpoints.
     *
     * @param distanceFeet Distance to target in feet
     * @return Optimal hood servo position (0.0 - 1.0)
     */
    public static double getHoodPositionForDistance(double distanceFeet) {
        // Handle edge cases
        if (distanceFeet <= 0 || Double.isNaN(distanceFeet)) {
            return HOOD_DEFAULT_POSITION;  // Default if no valid distance
        }

        // Clamp to table range
        if (distanceFeet <= DISTANCE_BREAKPOINTS[0]) {
            return HOOD_POSITIONS[0];
        }
        if (distanceFeet >= DISTANCE_BREAKPOINTS[DISTANCE_BREAKPOINTS.length - 1]) {
            return HOOD_POSITIONS[HOOD_POSITIONS.length - 1];
        }

        // Find the two breakpoints we're between
        for (int i = 0; i < DISTANCE_BREAKPOINTS.length - 1; i++) {
            double d1 = DISTANCE_BREAKPOINTS[i];
            double d2 = DISTANCE_BREAKPOINTS[i + 1];

            if (distanceFeet >= d1 && distanceFeet <= d2) {
                // Linear interpolation
                double t = (distanceFeet - d1) / (d2 - d1);
                double h1 = HOOD_POSITIONS[i];
                double h2 = HOOD_POSITIONS[i + 1];
                return h1 + t * (h2 - h1);
            }
        }

        return HOOD_DEFAULT_POSITION;
    }

    /**
     * Get flywheel power adjusted for distance.
     * Farther targets may need more power.
     *
     * @param distanceFeet Distance to target in feet
     * @return Recommended flywheel power (0.0 - 1.0)
     */
    public static double getFlywheelPowerForDistance(double distanceFeet) {
        // Simple linear scaling: closer = less power, farther = more power
        if (distanceFeet <= 0 || Double.isNaN(distanceFeet)) {
            return FLYWHEEL_SHOOTING_POWER;
        }

        // Scale from 0.7 at 2ft to 1.0 at 12ft
        double minDist = 2.0;
        double maxDist = 12.0;
        double minPower = 0.7;
        double maxPower = 1.0;

        if (distanceFeet <= minDist) return minPower;
        if (distanceFeet >= maxDist) return maxPower;

        double t = (distanceFeet - minDist) / (maxDist - minDist);
        return minPower + t * (maxPower - minPower);
    }
}

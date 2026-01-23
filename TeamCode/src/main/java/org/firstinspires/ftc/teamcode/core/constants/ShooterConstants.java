package org.firstinspires.ftc.teamcode.core.constants;

/**
 * *** MASTER SHOOTER CONSTANTS ***
 * ALL shooter and teleop settings are here - single source of truth!
 *
 * AFTER TUNING PIDF:
 * 1. Run FlywheelPIDFTuner, press B to save gains to telemetry log
 * 2. Copy the values from telemetry log
 * 3. Update the PIDF constants below
 * 4. Recompile and test in teleop
 */
public class ShooterConstants {

    private ShooterConstants() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== PIDF VELOCITY CONTROL (TUNE THESE!) ====================
    // Use FlywheelPIDFTuner to find optimal values, then update here
    // These values are used by BOTH the tuner and competition teleop

    public static final boolean USE_VELOCITY_CONTROL = true;  // Set false to use power mode (fallback)
    public static final double DEFAULT_TARGET_RPM = 3500;     // Default flywheel speed

    // PIDF Gains - TUNE USING FlywheelPIDFTuner!
    public static final double FLYWHEEL_KP = 0.00035;   // Proportional gain
    public static final double FLYWHEEL_KI = 0.000025;  // Integral gain
    public static final double FLYWHEEL_KD = 0.00015;   // Derivative gain
    public static final double FLYWHEEL_KF = 0.000215;  // Feedforward gain

    // ==================== FLYWHEEL POWER (Legacy - only used if velocity control OFF) ====================
    public static final double FLYWHEEL_DEFAULT_POWER = 0.8;
    public static final double FLYWHEEL_SHOOTING_POWER = 0.8;
    public static final double FLYWHEEL_LOW_POWER = 0.5;
    public static final double FLYWHEEL_MAX_POWER = 1.0;

    // ==================== HOOD POSITIONS ====================
    public static final double HOOD_DEFAULT_POSITION = 0.8;
    public static final double HOOD_MIN_POSITION = 0.5;    // Lowest angle (far shots)
    public static final double HOOD_MAX_POSITION = 0.9;    // Highest angle (close shots)
    public static final double HOOD_INCREMENT = 0.05;

    // ==================== INTAKE POWER ====================
    public static final double INTAKE_DEFAULT_POWER = -1;
    public static final double INTAKE_SHOOTING_POWER = 1.0;

    // ==================== TRANSFER POSITIONS ====================
    public static final double TRANSFER_UP_POSITION = 0.75;  // Max height - servo limit
    public static final double TRANSFER_DOWN_POSITION = 0.5;

    // ==================== LIMELIGHT DISTANCE-BASED SHOOTING PRESETS ====================
    // After testing at each distance, adjust these RPM values for accurate shooting
    // Range format: MIN distance, MAX distance, Target RPM, Hood position

    // Range 1: 2.47 - 2.84 ft (Close range)
    public static final double RANGE_1_MIN = 2.47;
    public static final double RANGE_1_MAX = 2.84;
    public static final double RANGE_1_FLYWHEEL_RPM = 2600;
    public static final double RANGE_1_HOOD_POSITION = 0.40;

    // Range 2: 2.84 - 3.2 ft (Close range)
    public static final double RANGE_2_MIN = 2.84;
    public static final double RANGE_2_MAX = 3.20;
    public static final double RANGE_2_FLYWHEEL_RPM = 2800;
    public static final double RANGE_2_HOOD_POSITION = 0.55;

    // Range 3: 3.21 - 4.0 ft (Close-Mid transition)
    public static final double RANGE_3_MIN = 3.21;
    public static final double RANGE_3_MAX = 4.00;
    public static final double RANGE_3_FLYWHEEL_RPM = 3000;
    public static final double RANGE_3_HOOD_POSITION = 0.50;

    // Range 4: 4.0 - 4.5 ft (Mid range)
    public static final double RANGE_4_MIN = 4.00;
    public static final double RANGE_4_MAX = 4.50;
    public static final double RANGE_4_FLYWHEEL_RPM = 3400;
    public static final double RANGE_4_HOOD_POSITION = 0.50;

    // Range 5: 4.6 - 5.0 ft (Mid range)
    public static final double RANGE_5_MIN = 4.60;
    public static final double RANGE_5_MAX = 5.00;
    public static final double RANGE_5_FLYWHEEL_RPM = 3600;
    public static final double RANGE_5_HOOD_POSITION = 0.70;

    // Range 6: 4.84 - 5.25 ft (Mid-Far transition)
    public static final double RANGE_6_MIN = 4.84;
    public static final double RANGE_6_MAX = 5.25;
    public static final double RANGE_6_FLYWHEEL_RPM = 3800;
    public static final double RANGE_6_HOOD_POSITION = 0.65;

    // Range 7: 5.2 - 5.7 ft (Far range)
    public static final double RANGE_7_MIN = 5.20;
    public static final double RANGE_7_MAX = 5.70;
    public static final double RANGE_7_FLYWHEEL_RPM = 4000;
    public static final double RANGE_7_HOOD_POSITION = 0.65;

    // Far shooting zone (beyond 5.7 ft) - Maximum distance on field
    public static final double RANGE_FAR_MIN = 5.70;
    public static final double RANGE_FAR_FLYWHEEL_RPM = 4500;
    public static final double RANGE_FAR_HOOD_POSITION = 0.67;

    // ==================== LIMELIGHT CAMERA SPECS ====================
    public static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032;  // 8 inches
    public static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;
    public static final int IMAGE_WIDTH_PIXELS = 1280;
    public static final int IMAGE_HEIGHT_PIXELS = 720;

    // ==================== DRIVE CONTROL SETTINGS ====================
    public static final double SPEED_SHOOTING_MULTIPLIER = 0.85;  // Reduce speed while flywheel on
    public static final double JOYSTICK_DEAD_ZONE = 0.05;
    public static final double TRIGGER_DEADZONE = 0.1;

    // ==================== TURRET MANUAL CONTROL ====================
    public static final double TURRET_MANUAL_INCREMENT = 0.02;          // DPAD increment
    public static final double TURRET_MANUAL_STICK_DEADZONE = 0.1;     // Stick deadzone
    public static final double TURRET_MANUAL_STICK_SENSITIVITY = 0.01; // Stick scaling
    public static final double TURRET_MIN_POSITION = 0.0;
    public static final double TURRET_MAX_POSITION = 1.0;
    public static final double TURRET_CENTER_POSITION = 0.3;

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

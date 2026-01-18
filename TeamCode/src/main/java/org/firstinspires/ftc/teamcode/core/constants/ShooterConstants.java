package org.firstinspires.ftc.teamcode.core.constants;

/**
 * Centralized shooter configuration constants.
 * Includes flywheel power, hood positions, and distance-based adjustments.
 */
public class ShooterConstants {

    private ShooterConstants() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== FLYWHEEL POWER ====================
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
    public static final double TRANSFER_UP_POSITION = 0.67;
    public static final double TRANSFER_DOWN_POSITION = 0.5;

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

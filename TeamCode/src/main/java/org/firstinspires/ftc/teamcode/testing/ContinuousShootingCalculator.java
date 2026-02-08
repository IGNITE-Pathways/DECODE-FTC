package org.firstinspires.ftc.teamcode.testing;

/**
 * Continuous Shooting Calculator
 *
 * Uses linear equations to calculate optimal shooting parameters based on distance.
 * This provides smooth, continuous adjustments instead of discrete range steps.
 *
 * EQUATIONS:
 * RPM(x) = 2950 + 160(x - 6.5)
 * Hood(x) = 0.72 + 0.008(x - 6.5)
 *
 * Where x = distance in feet
 *
 * BASELINE (x = 6.5 ft):
 * - RPM: 2950
 * - Hood: 0.72
 *
 * SLOPE:
 * - RPM increases by 160 per foot
 * - Hood increases by 0.008 per foot
 *
 * TUNING:
 * Adjust the constants below after field testing to fine-tune performance.
 */
public class ContinuousShootingCalculator {

    private ContinuousShootingCalculator() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== EQUATION PARAMETERS ====================
    // These are the coefficients for the linear equations
    // Tune these values after field testing for optimal accuracy

    // RPM equation: RPM(x) = RPM_BASELINE + RPM_SLOPE * (x - BASELINE_DISTANCE)
    private static final double RPM_BASELINE = 2950.0;      // RPM at baseline distance
    private static final double RPM_SLOPE = 160.0;          // RPM increase per foot
    private static final double BASELINE_DISTANCE = 6.5;    // Baseline distance (feet)

    // Hood equation: Hood(x) = HOOD_BASELINE + HOOD_SLOPE * (x - BASELINE_DISTANCE)
    private static final double HOOD_BASELINE = 0.72;       // Hood position at baseline
    private static final double HOOD_SLOPE = 0.008;         // Hood increase per foot

    // Physical limits (safety bounds)
    private static final double MIN_RPM = 1000.0;           // Minimum safe RPM
    private static final double MAX_RPM = 5400.0;           // Maximum motor RPM
    private static final double MIN_HOOD = 0.1;             // Minimum hood position
    private static final double MAX_HOOD = 0.9;             // Maximum hood position
    private static final double MIN_DISTANCE = 0.0;         // Minimum valid distance
    private static final double MAX_DISTANCE = 15.0;        // Maximum field distance

    // ==================== CALCULATION METHODS ====================

    /**
     * Calculate optimal RPM for a given distance using the linear equation.
     *
     * @param distanceFeet Distance to target in feet
     * @return Optimal flywheel RPM (clamped to safe limits)
     */
    public static double calculateRPM(double distanceFeet) {
        // Validate input
        if (!isValidDistance(distanceFeet)) {
            return RPM_BASELINE; // Return baseline for invalid input
        }

        // Calculate using equation: RPM(x) = 2950 + 160(x - 6.5)
        double rpm = RPM_BASELINE + RPM_SLOPE * (distanceFeet - BASELINE_DISTANCE);

        // Clamp to safe limits
        return clamp(rpm, MIN_RPM, MAX_RPM);
    }

    /**
     * Calculate optimal hood angle for a given distance using the linear equation.
     *
     * @param distanceFeet Distance to target in feet
     * @return Optimal hood position (clamped to safe limits)
     */
    public static double calculateHoodAngle(double distanceFeet) {
        // Validate input
        if (!isValidDistance(distanceFeet)) {
            return HOOD_BASELINE; // Return baseline for invalid input
        }

        // Calculate using equation: Hood(x) = 0.72 + 0.008(x - 6.5)
        double hood = HOOD_BASELINE + HOOD_SLOPE * (distanceFeet - BASELINE_DISTANCE);

        // Clamp to safe limits
        return clamp(hood, MIN_HOOD, MAX_HOOD);
    }

    /**
     * Calculate both RPM and hood angle for a given distance.
     *
     * @param distanceFeet Distance to target in feet
     * @return ShootingParameters with calculated RPM and hood angle
     */
    public static ShootingParameters calculate(double distanceFeet) {
        double rpm = calculateRPM(distanceFeet);
        double hood = calculateHoodAngle(distanceFeet);
        boolean valid = isValidDistance(distanceFeet);

        return new ShootingParameters(rpm, hood, distanceFeet, valid);
    }

    // ==================== VALIDATION METHODS ====================

    /**
     * Check if a distance is within valid range.
     */
    public static boolean isValidDistance(double distanceFeet) {
        return distanceFeet >= MIN_DISTANCE && distanceFeet <= MAX_DISTANCE && Double.isFinite(distanceFeet);
    }

    /**
     * Get the equation info as a string (for debugging/telemetry).
     */
    public static String getRPMEquation() {
        return String.format("RPM(x) = %.0f + %.0f(x - %.1f)", RPM_BASELINE, RPM_SLOPE, BASELINE_DISTANCE);
    }

    /**
     * Get the hood equation as a string (for debugging/telemetry).
     */
    public static String getHoodEquation() {
        return String.format("Hood(x) = %.2f + %.3f(x - %.1f)", HOOD_BASELINE, HOOD_SLOPE, BASELINE_DISTANCE);
    }

    // ==================== HELPER METHODS ====================

    /**
     * Clamp a value between min and max.
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ==================== DATA STRUCTURE ====================

    /**
     * Container for calculated shooting parameters.
     */
    public static class ShootingParameters {
        public final double rpm;
        public final double hoodAngle;
        public final double distanceFeet;
        public final boolean isValid;

        public ShootingParameters(double rpm, double hoodAngle, double distanceFeet, boolean isValid) {
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
            this.distanceFeet = distanceFeet;
            this.isValid = isValid;
        }

        @Override
        public String toString() {
            return String.format("Distance: %.2f ft → RPM: %.0f, Hood: %.3f %s",
                distanceFeet, rpm, hoodAngle, isValid ? "" : "(INVALID)");
        }
    }

    // ==================== EQUATION TESTING ====================

    /**
     * Generate a lookup table for testing/debugging.
     * Shows calculated values at different distances.
     *
     * @return Multi-line string with distance/RPM/hood values
     */
    public static String generateLookupTable() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== CONTINUOUS SHOOTING LOOKUP TABLE ===\n");
        sb.append(String.format("Equations:\n  %s\n  %s\n\n", getRPMEquation(), getHoodEquation()));
        sb.append("Dist(ft) | RPM  | Hood  \n");
        sb.append("---------|------|-------\n");

        // Generate table from 2 ft to 12 ft in 0.5 ft increments
        for (double dist = 2.0; dist <= 12.0; dist += 0.5) {
            ShootingParameters params = calculate(dist);
            sb.append(String.format("  %4.1f   | %4.0f | %.3f\n",
                params.distanceFeet, params.rpm, params.hoodAngle));
        }

        return sb.toString();
    }

    /**
     * Compare continuous equation values with discrete range values.
     * Useful for validating that equations match your tuned ranges.
     */
    public static String compareWithRanges() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== EQUATION vs RANGE COMPARISON ===\n\n");

        // Test points at the center of each original range
        double[][] rangeTests = {
            {2.95, 2400, 0.55},  // Range 1 center
            {4.05, 2600, 0.65},  // Range 2 center
            {5.00, 2700, 0.70},  // Range 3 center
            {5.68, 2800, 0.70},  // Range 4 center
            {6.50, 2950, 0.72},  // Range 5 center (baseline!)
            {7.75, 3150, 0.73},  // Range 6 center
            {9.25, 3300, 0.74},  // Range 7 center
            {10.5, 3400, 0.75}   // Far range
        };

        sb.append("Distance | Equation RPM | Range RPM | Diff | Equation Hood | Range Hood | Diff\n");
        sb.append("---------|--------------|-----------|------|---------------|------------|-----\n");

        for (double[] test : rangeTests) {
            double dist = test[0];
            double rangeRPM = test[1];
            double rangeHood = test[2];

            ShootingParameters calc = calculate(dist);
            double rpmDiff = calc.rpm - rangeRPM;
            double hoodDiff = calc.hoodAngle - rangeHood;

            sb.append(String.format("  %4.2f  |    %4.0f     |   %4.0f    | %+4.0f |    %.3f      |   %.2f     | %+.3f\n",
                dist, calc.rpm, rangeRPM, rpmDiff, calc.hoodAngle, rangeHood, hoodDiff));
        }

        return sb.toString();
    }
}

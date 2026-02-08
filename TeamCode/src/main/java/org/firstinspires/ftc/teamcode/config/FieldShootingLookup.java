package org.firstinspires.ftc.teamcode.config;

import java.util.ArrayList;
import java.util.List;

/**
 * Field Position-Based Shooting Lookup Table
 *
 * This class stores calibrated shooting settings for different positions on the field.
 * Use FieldShootingMapper.java to calibrate and add new positions.
 *
 * HOW TO USE:
 * ===========
 * 1. Run "Mapper: Field Shooting Positions" OpMode
 * 2. Drive to various shooting positions in launch zones
 * 3. Adjust and save optimal settings for each position
 * 4. Copy the logged data from telemetry
 * 5. Add new ShootingZone entries below
 * 6. In teleop, call getShootingSettings(x, y) to get optimal settings
 */
public class FieldShootingLookup {

    private FieldShootingLookup() {
        throw new IllegalStateException("Utility class");
    }

    // ==================== SHOOTING ZONES ====================
    // Add calibrated positions here after mapping with FieldShootingMapper

    private static final List<ShootingZone> SHOOTING_ZONES = new ArrayList<>();

    static {
        // Example zones - REPLACE WITH YOUR CALIBRATED DATA
        // Format: x, y, radius, flywheelRPM, hoodAngle

        // Close Left Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(24.0, 48.0, 12.0, 2400, 0.55));

        // Close Center Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(48.0, 48.0, 12.0, 2500, 0.57));

        // Close Right Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(72.0, 48.0, 12.0, 2400, 0.55));

        // Mid Left Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(24.0, 72.0, 12.0, 2700, 0.65));

        // Mid Center Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(48.0, 72.0, 12.0, 2800, 0.67));

        // Mid Right Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(72.0, 72.0, 12.0, 2700, 0.65));

        // Far Left Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(24.0, 96.0, 12.0, 3200, 0.72));

        // Far Center Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(48.0, 96.0, 12.0, 3400, 0.75));

        // Far Right Launch Zone
        SHOOTING_ZONES.add(new ShootingZone(72.0, 96.0, 12.0, 3200, 0.72));
    }

    // ==================== LOOKUP METHODS ====================

    /**
     * Get optimal shooting settings for a given field position.
     * Finds the closest calibrated zone within range.
     *
     * @param robotX Robot X position (inches)
     * @param robotY Robot Y position (inches)
     * @return ShootingSettings for this position, or null if no zone found
     */
    public static ShootingSettings getShootingSettings(double robotX, double robotY) {
        ShootingZone closestZone = null;
        double closestDistance = Double.MAX_VALUE;

        for (ShootingZone zone : SHOOTING_ZONES) {
            double distance = zone.distanceFrom(robotX, robotY);

            // Check if within zone radius and closer than previous best
            if (distance <= zone.radius && distance < closestDistance) {
                closestZone = zone;
                closestDistance = distance;
            }
        }

        if (closestZone != null) {
            return new ShootingSettings(
                closestZone.flywheelRPM,
                closestZone.hoodAngle,
                closestZone.name
            );
        }

        // No zone found - return defaults
        return new ShootingSettings(
            RobotConstants.DEFAULT_TARGET_RPM,
            RobotConstants.HOOD_DEFAULT_POSITION,
            "DEFAULT (No zone found)"
        );
    }

    /**
     * Check if robot is within any calibrated shooting zone.
     *
     * @param robotX Robot X position (inches)
     * @param robotY Robot Y position (inches)
     * @return true if in a calibrated zone
     */
    public static boolean isInShootingZone(double robotX, double robotY) {
        for (ShootingZone zone : SHOOTING_ZONES) {
            if (zone.contains(robotX, robotY)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get the name of the current shooting zone.
     *
     * @param robotX Robot X position (inches)
     * @param robotY Robot Y position (inches)
     * @return Zone name, or "Unknown Zone"
     */
    public static String getCurrentZoneName(double robotX, double robotY) {
        for (ShootingZone zone : SHOOTING_ZONES) {
            if (zone.contains(robotX, robotY)) {
                return zone.name;
            }
        }
        return "Unknown Zone";
    }

    /**
     * Add a new shooting zone (for runtime calibration).
     *
     * @param x Center X position (inches)
     * @param y Center Y position (inches)
     * @param radius Zone radius (inches)
     * @param rpm Optimal flywheel RPM
     * @param hood Optimal hood angle
     * @param name Zone name
     */
    public static void addShootingZone(double x, double y, double radius,
                                       double rpm, double hood, String name) {
        SHOOTING_ZONES.add(new ShootingZone(x, y, radius, rpm, hood, name));
    }

    /**
     * Get all shooting zones (for debugging/visualization).
     */
    public static List<ShootingZone> getAllZones() {
        return new ArrayList<>(SHOOTING_ZONES);
    }

    // ==================== DATA STRUCTURES ====================

    /**
     * Represents a calibrated shooting zone on the field.
     */
    public static class ShootingZone {
        public final double x;              // Center X position (inches)
        public final double y;              // Center Y position (inches)
        public final double radius;         // Zone radius (inches)
        public final double flywheelRPM;    // Optimal RPM
        public final double hoodAngle;      // Optimal hood angle
        public final String name;           // Zone name

        public ShootingZone(double x, double y, double radius,
                           double flywheelRPM, double hoodAngle) {
            this(x, y, radius, flywheelRPM, hoodAngle,
                 String.format("Zone (%.0f, %.0f)", x, y));
        }

        public ShootingZone(double x, double y, double radius,
                           double flywheelRPM, double hoodAngle, String name) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.flywheelRPM = flywheelRPM;
            this.hoodAngle = hoodAngle;
            this.name = name;
        }

        /**
         * Calculate distance from a point to the zone center.
         */
        public double distanceFrom(double px, double py) {
            double dx = px - x;
            double dy = py - y;
            return Math.sqrt(dx * dx + dy * dy);
        }

        /**
         * Check if a point is within this zone.
         */
        public boolean contains(double px, double py) {
            return distanceFrom(px, py) <= radius;
        }
    }

    /**
     * Shooting settings returned from lookup.
     */
    public static class ShootingSettings {
        public final double flywheelRPM;
        public final double hoodAngle;
        public final String zoneName;

        public ShootingSettings(double flywheelRPM, double hoodAngle, String zoneName) {
            this.flywheelRPM = flywheelRPM;
            this.hoodAngle = hoodAngle;
            this.zoneName = zoneName;
        }
    }
}

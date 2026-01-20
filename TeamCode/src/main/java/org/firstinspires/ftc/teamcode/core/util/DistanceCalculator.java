package org.firstinspires.ftc.teamcode.core.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.core.constants.TeleOpConstants;

/**
 * Utility class for calculating distance from Limelight AprilTag detection.
 * Caches constants for optimal performance.
 */
public class DistanceCalculator {

    private DistanceCalculator() {
        throw new IllegalStateException("Utility class");
    }

    // Pre-calculated constants (computed once at class load)
    private static final double FOCAL_LENGTH_PIXELS;
    private static final double TOTAL_IMAGE_PIXELS;
    private static final double METERS_TO_FEET = 3.28084;

    static {
        // Calculate focal length in pixels (this never changes)
        FOCAL_LENGTH_PIXELS = (TeleOpConstants.IMAGE_HEIGHT_PIXELS / 2.0)
            / Math.tan(Math.toRadians(TeleOpConstants.CAMERA_VERTICAL_FOV_DEGREES / 2.0));

        // Total pixels in image
        TOTAL_IMAGE_PIXELS = TeleOpConstants.IMAGE_WIDTH_PIXELS * TeleOpConstants.IMAGE_HEIGHT_PIXELS;
    }

    /**
     * Calculate distance to AprilTag from Limelight result.
     *
     * @param result The Limelight result containing tag area
     * @return Distance in feet, or -1.0 if unable to calculate
     */
    public static double calculateDistance(LLResult result) {
        if (result == null || !result.isValid()) {
            return -1.0;
        }

        double taPercent = result.getTa();
        if (taPercent <= 0.0) {
            return -1.0;
        }

        // Convert area percentage to pixel area
        double pixelArea = (taPercent / 100.0) * TOTAL_IMAGE_PIXELS;

        // Estimate tag height in pixels (assuming square tag)
        double tagPixelHeight = Math.sqrt(pixelArea);

        // Calculate distance using pinhole camera model
        double distanceMeters = (TeleOpConstants.APRILTAG_REAL_HEIGHT_METERS * FOCAL_LENGTH_PIXELS) / tagPixelHeight;

        // Convert to feet
        return distanceMeters * METERS_TO_FEET;
    }

    /**
     * Get the cached focal length in pixels.
     * Useful for debugging or advanced calculations.
     */
    public static double getFocalLengthPixels() {
        return FOCAL_LENGTH_PIXELS;
    }
}

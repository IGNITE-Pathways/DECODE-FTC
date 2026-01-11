package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

import java.util.List;

/**
 * Limelight Localization with GoBilda Pinpoint Odometry
 *
 * Uses Limelight's 3D AprilTag pose estimation for accurate distance
 * and Pinpoint odometry for robot position tracking.
 */
public class LimelightLocalization {

    // ==================== HARDWARE ====================
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;

    // ==================== PINPOINT CONFIGURATION ====================
    private static final double FORWARD_POD_Y = 3.75;
    private static final double STRAFE_POD_X = -7.08661;

    // ==================== ODOMETRY DATA ====================
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;
    private double headingOffset = 0;

    // ==================== VISION DATA ====================
    private int primaryTagId = -1;
    private double targetArea = 0;
    private double distanceInches = -1;      // 3D distance to tag in inches
    private double tx = 0;                    // Horizontal angle to target
    private double ty = 0;                    // Vertical angle to target
    private int tagCount = 0;
    private boolean tagVisible = false;

    // 3D pose data from AprilTag
    private double tagX = 0;                  // Tag X relative to camera (inches)
    private double tagY = 0;                  // Tag Y relative to camera (inches)
    private double tagZ = 0;                  // Tag Z relative to camera (inches)

    // ==================== TIMING ====================
    private double latencyMs = 0;

    // ==================== INITIALIZATION ====================

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT);

        if (limelight != null) {
            limelight.pipelineSwitch(3);
            limelight.setPollRateHz(100);
            limelight.start();
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
        if (pinpoint != null) {
            configurePinpoint();
        }

        telemetry.addLine("LimelightLocalization initialized");
        telemetry.addData("Limelight", limelight != null ? "OK" : "NOT FOUND");
        telemetry.addData("Pinpoint", pinpoint != null ? "OK" : "NOT FOUND");
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(STRAFE_POD_X, FORWARD_POD_Y, DistanceUnit.INCH);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
    }

    // ==================== MAIN UPDATE ====================

    public boolean update() {
        updatePinpoint();
        return updateVision();
    }

    private void updatePinpoint() {
        if (pinpoint == null) return;

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeading = pose.getHeading(AngleUnit.DEGREES) - headingOffset;
    }

    private boolean updateVision() {
        if (limelight == null || !limelight.isConnected()) {
            tagVisible = false;
            return false;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            tagVisible = false;
            return false;
        }

        latencyMs = result.getCaptureLatency() + result.getTargetingLatency();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            tagVisible = false;
            primaryTagId = -1;
            tagCount = 0;
            distanceInches = -1;
            return false;
        }

        tagCount = fiducials.size();
        tagVisible = true;

        // Get primary tag data
        LLResultTypes.FiducialResult primary = fiducials.get(0);
        primaryTagId = primary.getFiducialId();
        tx = primary.getTargetXDegrees();
        ty = primary.getTargetYDegrees();
        targetArea = result.getTa();

        // Get 3D pose from AprilTag (camera-relative position)
        Pose3D cameraPose = primary.getCameraPoseTargetSpace();
        if (cameraPose != null) {
            Position pos = cameraPose.getPosition();
            // Limelight returns in meters, convert to inches
            tagX = pos.x * 39.3701;  // meters to inches
            tagY = pos.y * 39.3701;
            tagZ = pos.z * 39.3701;

            // Calculate 3D distance (Euclidean distance)
            distanceInches = Math.sqrt(tagX * tagX + tagY * tagY + tagZ * tagZ);
        } else {
            // Fallback: use target area formula if 3D pose not available
            distanceInches = calculateDistanceFromArea(targetArea);
        }

        return true;
    }

    // ==================== DISTANCE CALCULATION (FALLBACK) ====================

    /**
     * Fallback distance calculation using target area
     * Uses your calibrated equation: 5.35 + (-1.9 * area) + (0.353 * area^2)
     * Returns distance in inches
     */
    private double calculateDistanceFromArea(double area) {
        if (area <= 0) return -1;

        // Your calibrated equation (assumes result is in feet, convert to inches)
        double distanceFeet = 5.35 + (-1.9 * area) + (0.353 * area * area);
        return Math.max(0, distanceFeet * 12.0);  // Convert feet to inches
    }

    public double getDistanceToTag(int tagId) {
        if (!tagVisible) return -1;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return -1;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == tagId) {
                // Try to get 3D pose distance
                Pose3D pose = fr.getCameraPoseTargetSpace();
                if (pose != null) {
                    Position pos = pose.getPosition();
                    double x = pos.x * 39.3701;
                    double y = pos.y * 39.3701;
                    double z = pos.z * 39.3701;
                    return Math.sqrt(x * x + y * y + z * z);
                }
                // Fallback to area calculation
                return calculateDistanceFromArea(result.getTa());
            }
        }
        return -1;
    }

    // ==================== GETTERS - 3D POSE ====================

    public double getTagX() { return tagX; }
    public double getTagY() { return tagY; }
    public double getTagZ() { return tagZ; }

    // ==================== PINPOINT METHODS ====================

    public void resetPosition() {
        if (pinpoint != null) {
            pinpoint.setPosition(
                    new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)
            );
            headingOffset = 0;
        }
    }

    public void resetHeading() {
        if (pinpoint != null) {
            Pose2D pose = pinpoint.getPosition();
            headingOffset = pose.getHeading(AngleUnit.DEGREES);
        }
    }

    public void setHeading(double heading) {
        if (pinpoint != null) {
            Pose2D pose = pinpoint.getPosition();
            headingOffset = pose.getHeading(AngleUnit.DEGREES) - heading;
        }
    }

    public void setPosition(double x, double y, double heading) {
        if (pinpoint != null) {
            pinpoint.setPosition(
                    new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading)
            );
            headingOffset = 0;
        }
    }

    public void recalibrateIMU() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            headingOffset = 0;
        }
    }

    // ==================== GETTERS ====================

    public double getX() { return robotX; }
    public double getY() { return robotY; }
    public double getHeading() { return robotHeading; }

    public boolean isTagVisible() { return tagVisible; }
    public int getPrimaryTagId() { return primaryTagId; }
    public double getTargetArea() { return targetArea; }
    public double getDistance() { return distanceInches; } // inches
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getTagCount() { return tagCount; }

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    public boolean isPinpointConnected() {
        return pinpoint != null;
    }

    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        if (pinpoint == null) return null;
        return pinpoint.getDeviceStatus();
    }

    public double getLatencyMs() { return latencyMs; }

    public boolean isTagVisible(int tagId) {
        if (!tagVisible) return false;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return false;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == tagId) return true;
        }
        return false;
    }

    public double getAngleToTag(int tagId) {
        if (!tagVisible) return 0;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return 0;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == tagId) {
                return fr.getTargetXDegrees();
            }
        }
        return 0;
    }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        telemetry.addLine("=== LOCALIZATION ===");
        telemetry.addLine("");

        telemetry.addLine("--- PINPOINT ---");
        telemetry.addData("X", "%.1f in", robotX);
        telemetry.addData("Y", "%.1f in", robotY);
        telemetry.addData("Heading", "%.1f°", robotHeading);
        if (pinpoint != null) {
            telemetry.addData("Status", pinpoint.getDeviceStatus());
        }
        telemetry.addLine("");

        telemetry.addLine("--- APRILTAG ---");
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
        if (tagVisible) {
            telemetry.addData("Tag ID", primaryTagId);
            telemetry.addData("Distance", "%.1f in (%.2f ft)", distanceInches, distanceInches / 12.0);
            telemetry.addData("Angle (tx)", "%.1f°", tx);
            telemetry.addData("3D Pos", "X:%.1f Y:%.1f Z:%.1f", tagX, tagY, tagZ);
            telemetry.addData("Target Area", "%.3f%%", targetArea);
        }
        telemetry.addLine("");

        telemetry.addData("Tags Visible", tagCount);
        telemetry.addData("Latency", "%.0f ms", latencyMs);
    }
}

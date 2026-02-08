package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

import java.util.List;

/**
 * Geometric turret auto-lock using pure trigonometry (no PID)
 *
 * Uses 3D pose data from Limelight to calculate exact turret angle:
 * - angle = atan2(tagX, tagZ) where tagX=horizontal, tagZ=forward
 * - Direct servo positioning with exponential moving average smoothing
 * - Faster convergence than PID, no oscillation
 *
 * Calibration:
 * - SERVO_DEGREES_PER_UNIT determines servo-to-angle mapping
 * - Tune this value until physical turret angle matches calculated angle
 * - Use GeometricTurretTest.java to adjust in real-time
 */
public class GeometricTurretLock {

    // Hardware
    private Servo turretServo;
    private Limelight3A limelight;
    private Telemetry telemetry;

    // Servo limits (matching existing turret system)
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.9;
    private static final double SERVO_CENTER = 0.5;

    // Calibration - TUNE THIS ON ACTUAL ROBOT
    // Physical turret range: ~80° total (±40° from center)
    // Servo range: 0.8 units (0.9 - 0.1)
    // Initial estimate: 80° / 0.8 units = 100 deg/unit
    private double servoDegPerUnit = 100.0;

    // Smoothing filter (exponential moving average)
    // Higher = faster response but more jitter (0.0 to 1.0)
    // Lower = smoother but slower tracking
    private double smoothingAlpha = 0.4;

    // Locked detection tolerance
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;

    // Movement deadzone - prevent micro-adjustments that cause oscillation
    private static final double SERVO_MOVEMENT_DEADZONE = 0.002;  // Don't move if change < 0.002

    // Rate limiting - max servo change per update (prevents sudden jumps)
    private static final double MAX_SERVO_CHANGE_PER_UPDATE = 0.015;  // Max 0.015 per cycle

    // Target lost timeout
    private static final double TARGET_LOST_TIMEOUT = 0.5; // seconds

    // AprilTag configuration
    private int targetTagId = 20;  // Blue = 20, Red = 24
    private static final int APRILTAG_PIPELINE = 3;
    private static final int BLUE_TAG = 20;
    private static final int RED_TAG = 24;

    // State
    private double servoPos = SERVO_CENTER;
    private double smoothedServoPos = SERVO_CENTER;
    private double calculatedAngle = 0.0;
    private double angleError = 0.0;
    private boolean targetVisible = false;
    private boolean isLocked = false;
    private ElapsedTime targetLostTimer = new ElapsedTime();

    // 3D pose data (for telemetry)
    private double tagX = 0.0;  // Horizontal offset (meters)
    private double tagY = 0.0;  // Vertical offset (meters)
    private double tagZ = 0.0;  // Forward distance (meters)

    // Debug state
    private String debugState = "INIT";

    // ==================== INITIALIZATION ====================

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        initialize(hardwareMap, telemetry, AllianceColor.BLUE);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance) {
        this.telemetry = telemetry;
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;

        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            if (turretServo != null) {
                turretServo.setPosition(SERVO_CENTER);
                servoPos = SERVO_CENTER;
                smoothedServoPos = SERVO_CENTER;
            }

            limelight = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT);
            if (limelight != null) {
                limelight.pipelineSwitch(APRILTAG_PIPELINE); // Pipeline 3 for AprilTag
                limelight.setPollRateHz(100);
                limelight.start();
            }

            targetLostTimer.reset();

            debugState = "Initialized";
            if (telemetry != null) {
                telemetry.addData("Status", "Geometric Turret Initialized");
                telemetry.addData("Alliance", alliance.name());
                telemetry.addData("Target Tag", targetTagId);
                telemetry.update();
            }
        } catch (Exception e) {
            debugState = "Init Error: " + e.getMessage();
            if (telemetry != null) {
                telemetry.addData("Init Error", e.getMessage());
            }
        }
    }

    // ==================== MAIN UPDATE LOOP ====================

    /**
     * Main update loop - call this continuously
     * @return true if target is visible and tracking, false otherwise
     */
    public boolean update() {
        try {
            if (limelight == null || !limelight.isConnected()) {
                debugState = "NO LIMELIGHT";
                targetVisible = false;
                isLocked = false;
                return false;
            }

            LLResult result = limelight.getLatestResult();

            // Check for valid result with fiducials
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    // Look for target AprilTag
                    LLResultTypes.FiducialResult targetFiducial = null;

                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        if (fr.getFiducialId() == targetTagId) {
                            targetFiducial = fr;
                            break;
                        }
                    }

                    if (targetFiducial != null) {
                        // TARGET FOUND!
                        targetVisible = true;
                        targetLostTimer.reset();

                        // Get 3D pose from Limelight
                        Pose3D cameraPose = targetFiducial.getCameraPoseTargetSpace();

                        if (cameraPose != null) {
                            Position pos = cameraPose.getPosition();

                            // Convert from meters (Limelight) to stored values
                            tagX = pos.x;  // Horizontal offset (positive = right)
                            tagY = pos.y;  // Vertical offset
                            tagZ = pos.z;  // Forward distance

                            // CORE TRIGONOMETRY: Calculate angle to target
                            calculatedAngle = calculateAngleToTarget(tagX, tagZ);

                            // Map angle to servo position
                            double targetServoPos = angleToServoPosition(calculatedAngle);

                            // Apply exponential moving average smoothing
                            smoothedServoPos = (smoothingAlpha * targetServoPos) +
                                             ((1.0 - smoothingAlpha) * smoothedServoPos);

                            // Clamp to servo limits
                            smoothedServoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, smoothedServoPos));

                            // Calculate angle error for lock detection
                            angleError = Math.abs(calculatedAngle);

                            // Check if locked (within tolerance)
                            if (angleError < ANGLE_TOLERANCE_DEGREES) {
                                isLocked = true;
                                debugState = "LOCKED ON";
                                // CRITICAL: Stop moving when locked to prevent oscillation
                                // Keep current position, don't update
                            } else {
                                isLocked = false;
                                debugState = String.format("TRACKING (angle=%.1f°)", calculatedAngle);

                                // Calculate how much servo would move
                                double servoChange = smoothedServoPos - servoPos;

                                // Apply movement deadzone - ignore tiny movements
                                if (Math.abs(servoChange) > SERVO_MOVEMENT_DEADZONE) {
                                    // Apply rate limiting - prevent sudden large movements
                                    if (Math.abs(servoChange) > MAX_SERVO_CHANGE_PER_UPDATE) {
                                        servoChange = Math.signum(servoChange) * MAX_SERVO_CHANGE_PER_UPDATE;
                                    }

                                    // Apply the limited movement
                                    servoPos += servoChange;
                                }
                                // else: movement too small, don't update (prevents jitter)
                            }

                        } else {
                            // 3D pose not available - fallback to TX angle
                            double tx = targetFiducial.getTargetXDegrees();
                            calculatedAngle = tx;
                            double targetServoPos = angleToServoPosition(tx);
                            smoothedServoPos = (smoothingAlpha * targetServoPos) +
                                             ((1.0 - smoothingAlpha) * smoothedServoPos);
                            smoothedServoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, smoothedServoPos));

                            angleError = Math.abs(tx);

                            // Same deadzone and rate limiting logic
                            if (angleError < ANGLE_TOLERANCE_DEGREES) {
                                isLocked = true;
                                debugState = "LOCKED (TX)";
                            } else {
                                isLocked = false;
                                debugState = "TRACKING (TX fallback)";

                                double servoChange = smoothedServoPos - servoPos;
                                if (Math.abs(servoChange) > SERVO_MOVEMENT_DEADZONE) {
                                    if (Math.abs(servoChange) > MAX_SERVO_CHANGE_PER_UPDATE) {
                                        servoChange = Math.signum(servoChange) * MAX_SERVO_CHANGE_PER_UPDATE;
                                    }
                                    servoPos += servoChange;
                                }
                            }
                        }

                    } else {
                        // Target tag not found
                        handleNoTarget();
                    }
                } else {
                    // No fiducials detected
                    handleNoTarget();
                }
            } else {
                // Invalid result
                handleNoTarget();
            }

            // Apply servo position
            if (turretServo != null && Double.isFinite(servoPos)) {
                double clampedPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(clampedPos);
            }

            return targetVisible;

        } catch (Exception e) {
            debugState = "Error: " + e.getMessage();
            targetVisible = false;
            isLocked = false;
            return false;
        }
    }

    // ==================== TRIGONOMETRY CALCULATIONS ====================

    /**
     * Calculate horizontal angle to target using atan2
     * @param x Horizontal offset (meters, positive = right)
     * @param z Forward distance (meters)
     * @return Angle in degrees (positive = right, negative = left)
     */
    private double calculateAngleToTarget(double x, double z) {
        // Handle edge case: target directly overhead or at zero distance
        if (Math.abs(z) < 0.001) {
            // Target too close or invalid, return center
            return 0.0;
        }

        // atan2(y, x) gives angle from positive x-axis
        // We use atan2(x, z) to get horizontal angle from center
        double angleRadians = Math.atan2(x, z);
        double angleDegrees = Math.toDegrees(angleRadians);

        return angleDegrees;
    }

    /**
     * Map angle (degrees) to servo position
     * @param angleDegrees Angle in degrees (0 = center, + = right, - = left)
     * @return Servo position (0.1 to 0.9)
     */
    private double angleToServoPosition(double angleDegrees) {
        // Linear mapping: servoPos = center + (angle / degreesPerUnit)
        // Example: angle = +20°, degPerUnit = 100
        //          servoPos = 0.5 + (20 / 100) = 0.5 + 0.2 = 0.7

        double servoOffset = angleDegrees / servoDegPerUnit;
        double position = SERVO_CENTER + servoOffset;

        // Clamp to valid range
        return Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
    }

    // ==================== TARGET LOSS HANDLING ====================

    private void handleNoTarget() {
        if (targetVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
            // Recently lost target - hold position
            debugState = String.format("TARGET LOST - Holding (%.1fs)", targetLostTimer.seconds());
            isLocked = false;
            // Don't change servo position, keep holding
        } else {
            // Target lost for too long - return to center
            debugState = "SEARCHING";
            targetVisible = false;
            isLocked = false;

            // Smoothly return to center
            smoothedServoPos = (0.1 * SERVO_CENTER) + (0.9 * smoothedServoPos);
            servoPos = smoothedServoPos;
        }
    }

    // ==================== PUBLIC CONTROL METHODS ====================

    public void stop() {
        try {
            if (limelight != null) {
                limelight.stop();
            }
            targetVisible = false;
            isLocked = false;
        } catch (Exception e) {
            // Fail silently
        }
    }

    public void resetLock() {
        servoPos = SERVO_CENTER;
        smoothedServoPos = SERVO_CENTER;
        calculatedAngle = 0.0;
        angleError = 0.0;
        targetVisible = false;
        isLocked = false;
        targetLostTimer.reset();

        if (turretServo != null) {
            turretServo.setPosition(SERVO_CENTER);
        }
    }

    public void setAlliance(AllianceColor alliance) {
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;
    }

    public void setPositionDirect(double position) {
        if (!Double.isFinite(position)) {
            return;
        }

        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
        smoothedServoPos = servoPos;

        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }
    }

    // ==================== TUNING METHODS ====================

    /**
     * Adjust servo calibration (degrees per servo unit)
     * @param degPerUnit Degrees of physical rotation per 1.0 servo units
     */
    public void setServoDegPerUnit(double degPerUnit) {
        if (degPerUnit > 0 && Double.isFinite(degPerUnit)) {
            this.servoDegPerUnit = degPerUnit;
        }
    }

    public void adjustServoDegPerUnit(double delta) {
        servoDegPerUnit = Math.max(10.0, Math.min(200.0, servoDegPerUnit + delta));
    }

    /**
     * Adjust smoothing alpha (0.0 = max smoothing, 1.0 = no smoothing)
     * @param alpha Smoothing factor
     */
    public void setSmoothingAlpha(double alpha) {
        this.smoothingAlpha = Math.max(0.0, Math.min(1.0, alpha));
    }

    public void adjustSmoothingAlpha(double delta) {
        smoothingAlpha = Math.max(0.0, Math.min(1.0, smoothingAlpha + delta));
    }

    // ==================== GETTERS ====================

    public boolean isLocked() {
        return isLocked;
    }

    public double getServoPosition() {
        return servoPos;
    }

    public double getSmoothedServoPosition() {
        return smoothedServoPos;
    }

    public double getCalculatedAngle() {
        return calculatedAngle;
    }

    public double getAngleError() {
        return angleError;
    }

    public String getDebugState() {
        return debugState;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    public boolean isLimelightConnected() {
        return limelight != null && limelight.isConnected();
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public double getServoDegPerUnit() {
        return servoDegPerUnit;
    }

    public double getSmoothingAlpha() {
        return smoothingAlpha;
    }

    // 3D pose getters for telemetry
    public double getTagX() {
        return tagX;
    }

    public double getTagY() {
        return tagY;
    }

    public double getTagZ() {
        return tagZ;
    }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== GEOMETRIC TURRET (TRIGONOMETRY) ===");
        telemetry.addData("Status", debugState);
        telemetry.addData("Target Tag", targetTagId);
        telemetry.addData("Locked", isLocked ? "YES" : "no");
        telemetry.addLine("");

        if (targetVisible) {
            telemetry.addLine("--- 3D Position Data ---");
            telemetry.addData("Tag X", "%.3fm (horiz)", tagX);
            telemetry.addData("Tag Z", "%.3fm (forward)", tagZ);
            telemetry.addData("Calculated Angle", "%.2f°", calculatedAngle);
            telemetry.addData("Angle Error", "%.2f°", angleError);
            telemetry.addLine("");
        }

        telemetry.addLine("--- Servo Control ---");
        telemetry.addData("Target Servo", "%.3f", angleToServoPosition(calculatedAngle));
        telemetry.addData("Smoothed Servo", "%.3f", smoothedServoPos);
        telemetry.addData("Current Servo", "%.3f", servoPos);
        telemetry.addLine("");

        telemetry.addLine("--- Tuning Parameters ---");
        telemetry.addData("Deg/ServoUnit", "%.1f", servoDegPerUnit);
        telemetry.addData("Smoothing Alpha", "%.2f", smoothingAlpha);
        telemetry.addLine("");

        telemetry.addData("Limelight", isLimelightConnected() ? "CONNECTED" : "NOT CONNECTED");
    }
}

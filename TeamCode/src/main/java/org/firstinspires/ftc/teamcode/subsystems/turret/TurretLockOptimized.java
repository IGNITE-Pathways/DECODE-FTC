package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import java.util.List;

/**
 * Auto-aiming turret for servo with integrated drive
 * Includes Limelight TX offset compensation for camera mounting position
 */
public class TurretLockOptimized {

    // Hardware - Turret
    private Servo turretServo;
    private Limelight3A limelight;
    private Telemetry telemetry;

    // Hardware - Drive
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private boolean driveInitialized = false;

    // Limelight mounting offset - ADJUST THIS TO MATCH YOUR CAMERA MOUNTING ANGLE
    // Positive value = camera points to the right of robot center
    // Negative value = camera points to the left of robot center
    private static final double LIMELIGHT_TX_OFFSET_DEGREES = 5.0;  // Adjust based on mounting

    // PID Controller variables for SERVO - Values loaded from RobotConstants
    private double kP = RobotConstants.TURRET_KP;   // Proportional gain - reduced to prevent overshoot
    private double kI = 0.0;                        // Integral gain - DISABLED to prevent windup oscillation
    private double kD = RobotConstants.TURRET_KD;   // Derivative gain - increased damping to prevent oscillation

    private double targetX = 0.0; // Target is centered (tx = 0)
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    // Low-pass filter for tx (prevents jitter-induced oscillation)
    private double filteredTX = 0.0;
    private boolean filterInitialized = false;  // Track if filter has been seeded

    // Deadband and limits
    private static final double MIN_ADJUSTMENT = 0.0002;  // Minimum servo move
    private static final double MAX_ADJUSTMENT = 0.018;   // Increased for faster tracking
    private static final double TARGET_LOST_TIMEOUT = 0.5; // seconds before scanning


    // Servo limits - Loaded from RobotConstants
    private static final double SERVO_MIN = RobotConstants.TURRET_MIN_POSITION;
    private static final double SERVO_MAX = RobotConstants.TURRET_MAX_POSITION;
    private static final double SERVO_CENTER = RobotConstants.TURRET_CENTER_POSITION;

    // Direction tuning - CHANGE THIS IF TURRET MOVES WRONG WAY
    private static final boolean INVERT_DIRECTION = false;

    // Scanning
    private static final double SCAN_SPEED = 0.001;
    private int scanDirection = 1;

    // State
    private double servoPos = SERVO_CENTER;
    private boolean targetWasVisible = false;
    private boolean isLocked = false;
    private boolean tagVisible = false;
    private boolean manualLockOverride = false;  // Manual lock/unlock control

    // Target tag
    private int targetTagId = 20;  // Blue = 20, Red = 24
    private static final int APRILTAG_PIPELINE = 3;
    private static final int BLUE_TAG = 20;
    private static final int RED_TAG = 24;

    // Debug
    private double currentTX = 0.0;
    private double currentError = 0.0;
    private double pidOutput = 0.0;
    private String debugState = "INIT";
    private String detectedTagIds = "none";
    private int detectedTagCount = 0;

    private double currentTY = 0.0;            // Current vertical offset

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        initialize(hardwareMap, telemetry, AllianceColor.BLUE);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance) {
        this.telemetry = telemetry;
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;

        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            if (turretServo != null) {
                turretServo.setPosition(servoPos);
            }

            limelight = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT);
            if (limelight != null) {
                limelight.pipelineSwitch(APRILTAG_PIPELINE);
                limelight.setPollRateHz(100);
                limelight.start();
            }

            // Initialize drive motors for auto-strafing
            try {
                frontLeft = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_FRONT_MOTOR);
                frontRight = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_FRONT_MOTOR);
                backLeft = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_BACK_MOTOR);
                backRight = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_BACK_MOTOR);

                // Set motor directions for mecanum drive
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.FORWARD);

                driveInitialized = true;
            } catch (Exception e) {
                driveInitialized = false;
                // Continue even if drive initialization fails
            }

            timer.reset();
            targetLostTimer.reset();

            debugState = "Initialized";
        } catch (Exception e) {
            debugState = "Init Error: " + e.getMessage();
        }
    }

    public boolean update() {
        try {
            if (limelight == null || !limelight.isConnected()) {
                debugState = "NO LIMELIGHT";
                return false;
            }

            LLResult result = limelight.getLatestResult();

            // Check if we have a valid result with fiducial data
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    // Look for our target tag
                    LLResultTypes.FiducialResult targetFiducial = null;
                    StringBuilder ids = new StringBuilder();

                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        if (ids.length() > 0) ids.append(", ");
                        ids.append(fr.getFiducialId());

                        if (fr.getFiducialId() == targetTagId) {
                            targetFiducial = fr;
                        }
                    }
                    detectedTagIds = ids.toString();
                    detectedTagCount = fiducials.size();

                    if (targetFiducial != null) {
                        // TARGET FOUND!
                        boolean firstAcquisition = !tagVisible;  // Track if this is first time seeing target
                        tagVisible = true;
                        targetWasVisible = true;
                        targetLostTimer.reset();

                        // Get horizontal offset from center (tx) and vertical offset (ty)
                        double rawTx = targetFiducial.getTargetXDegrees();
                        double ty = targetFiducial.getTargetYDegrees();
                        currentTY = ty;

                        // Apply limelight mounting offset compensation
                        // This corrects for the camera not pointing directly at the target
                        double compensatedTx = rawTx - LIMELIGHT_TX_OFFSET_DEGREES;

                        // Apply low-pass filter to tx to reduce jitter
                        // Initialize filter on first reading to prevent large jump
                        if (!filterInitialized || firstAcquisition) {
                            filteredTX = compensatedTx;
                            filterInitialized = true;
                            timer.reset();  // Reset timer on first acquisition
                        } else {
                            filteredTX = (RobotConstants.TURRET_TX_FILTER_ALPHA * compensatedTx) +
                                        ((1.0 - RobotConstants.TURRET_TX_FILTER_ALPHA) * filteredTX);
                        }
                        currentTX = filteredTX;

                        // Calculate time since last update
                        double dt = timer.seconds();
                        timer.reset();

                        // Prevent division by zero or huge derivatives
                        if (dt < 0.001) dt = 0.001;
                        if (dt > 0.5) dt = 0.5;  // Cap dt to prevent huge jumps

                        // Calculate error using filtered value
                        double error = filteredTX - targetX;
                        currentError = error;

                        // ========== SERVO CONTROL ==========
                        // CONTINUOUS TRACKING: Always update servo to follow target, even when "locked"
                        // "Locked" just means we're within tolerance, but we keep tracking for robot movement

                        // Check if within tolerance for status indicator (unless manual override is set)
                        if (!manualLockOverride) {
                            if (Math.abs(error) < RobotConstants.TURRET_POSITION_TOLERANCE) {
                                isLocked = true;
                                debugState = "LOCKED ON (tracking)";
                            } else {
                                isLocked = false;
                                debugState = "TRACKING tx=" + String.format("%.1f", filteredTX);
                            }
                        } else {
                            debugState = "MANUAL LOCK OVERRIDE";
                        }

                        // PID calculations - ALWAYS applied for continuous tracking
                        double derivative = (error - lastError) / dt;
                        pidOutput = (kP * error) + (kD * derivative);

                        // Clamp derivative contribution to prevent spikes
                        double maxDerivative = 0.015;
                        double derivativeContrib = kD * derivative;
                        if (Math.abs(derivativeContrib) > maxDerivative) {
                            derivativeContrib = maxDerivative * Math.signum(derivativeContrib);
                            pidOutput = (kP * error) + derivativeContrib;
                        }

                        // Apply minimum adjustment threshold only if error is significant
                        if (Math.abs(error) > 0.5 && Math.abs(pidOutput) < MIN_ADJUSTMENT) {
                            pidOutput = MIN_ADJUSTMENT * Math.signum(pidOutput);
                        }

                        lastError = error;

                        // Clamp output to max adjustment
                        double servoAdjust = Math.max(-MAX_ADJUSTMENT, Math.min(MAX_ADJUSTMENT, pidOutput));

                        // Micro-deadzone: ignore tiny adjustments to prevent jitter (only when locked)
                        if (isLocked && Math.abs(servoAdjust) < 0.0001) {
                            servoAdjust = 0;
                        }

                        // Apply inversion if needed
                        if (INVERT_DIRECTION) {
                            servoAdjust = -servoAdjust;
                        }

                        // Safety check
                        if (!Double.isFinite(servoAdjust)) {
                            servoAdjust = 0;
                            resetPID();
                        }

                        // Apply to servo position
                        servoPos += servoAdjust;
                        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));

                    } else {
                        // Tags detected but not our target tag
                        handleNoTarget();
                    }
                } else {
                    // No fiducials detected
                    detectedTagCount = 0;
                    detectedTagIds = "none";
                    handleNoTarget();
                }
            } else {
                // No valid result
                detectedTagCount = 0;
                detectedTagIds = "no result";
                handleNoTarget();
            }

            // Apply servo position with final safety check
            if (turretServo != null && Double.isFinite(servoPos)) {
                double clampedPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(clampedPos);
            }

            return true;

        } catch (Exception e) {
            debugState = "Error: " + e.getMessage();
            return false;
        }
    }

    private void handleNoTarget() {
        tagVisible = false;
        manualLockOverride = false;  // Clear manual override when target is lost

        if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
            // Recently lost target - hold position (coast)
            debugState = "HOLDING (lost " + String.format("%.1fs", targetLostTimer.seconds()) + ")";
            isLocked = false;
        } else {
            // Target lost for too long - scan
            debugState = "SCANNING";
            isLocked = false;
            targetWasVisible = false;
            resetPID();

            // Scan back and forth
            servoPos += scanDirection * SCAN_SPEED;
            if (servoPos >= SERVO_MAX - 0.02) {
                scanDirection = -1;
            } else if (servoPos <= SERVO_MIN + 0.02) {
                scanDirection = 1;
            }
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
        }
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        pidOutput = 0;
        filteredTX = 0;
        filterInitialized = false;  // Reset filter on next reading
    }

    // ==================== DRIVE CONTROL METHODS ====================

    /**
     * Manual drive control for mecanum drivetrain.
     * Call this method from your teleop loop to drive while the turret tracks.
     *
     * @param strafe   Left/right movement (-1.0 to 1.0, positive = right)
     * @param forward  Forward/backward movement (-1.0 to 1.0, positive = forward)
     * @param rotate   Rotation (-1.0 to 1.0, positive = clockwise)
     */
    public void drive(double strafe, double forward, double rotate) {
        if (!driveInitialized || frontLeft == null || frontRight == null ||
            backLeft == null || backRight == null) {
            return;
        }

        // Mecanum drive kinematics
        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                                   Math.max(Math.abs(blPower), Math.abs(brPower)));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    /**
     * Stop all drive motors.
     */
    public void stopDrive() {
        if (!driveInitialized) return;

        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    /**
     * Check if drive motors are initialized and ready to use.
     * @return true if drive motors are available
     */
    public boolean isDriveInitialized() {
        return driveInitialized;
    }

    // ==================== PUBLIC METHODS ====================

    /**
     * Stop the limelight and turret tracking.
     * Does not stop drive motors - use stopDrive() for that.
     */
    public void stop() {
        try {
            if (limelight != null) {
                limelight.stop();
            }
            resetPID();
            tagVisible = false;
            targetWasVisible = false;
            isLocked = false;
        } catch (Exception e) {
            // Fail silently
        }
    }

    /**
     * Complete shutdown - stops both turret tracking and drive motors.
     */
    public void shutdown() {
        stop();
        stopDrive();
    }

    /**
     * Reset the turret to center position and clear all tracking state.
     * Use this to reinitialize tracking or when switching modes.
     */
    public void resetLock() {
        resetPID();
        servoPos = SERVO_CENTER;
        targetWasVisible = false;
        isLocked = false;
        tagVisible = false;
        manualLockOverride = false;
        timer.reset();
        targetLostTimer.reset();
        currentTX = 0;
        currentTY = 0;
        currentError = 0;
        detectedTagIds = "none";
        detectedTagCount = 0;
    }

    /**
     * Set the alliance color to track the correct AprilTag.
     * Blue = Tag 20, Red = Tag 24
     */
    public void setAlliance(AllianceColor alliance) {
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;
        resetPID();  // Reset tracking when changing targets
    }

    /**
     * Get the current limelight TX offset being applied.
     * @return Offset in degrees
     */
    public double getLimelightOffset() {
        return LIMELIGHT_TX_OFFSET_DEGREES;
    }

    /**
     * Set turret servo position directly, bypassing tracking.
     * @param position Servo position (0.0 to 1.0), will be clamped to valid range
     */
    public void setPositionDirect(double position) {
        if (!Double.isFinite(position)) {
            return;  // Ignore invalid values
        }

        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }

        // Reset tracking when manually positioning
        resetPID();
        manualLockOverride = false;
    }

    /**
     * Manually force the turret to be "locked" regardless of tracking error.
     * Note: This doesn't stop servo movement, just sets the status flag.
     */
    public void lock() {
        isLocked = true;
        manualLockOverride = true;
    }

    /**
     * Manually force the turret to be "unlocked".
     * Note: Normal auto-tracking will resume on next update().
     */
    public void unlock() {
        isLocked = false;
        manualLockOverride = false;
    }

    /**
     * Set PID gains for servo control.
     * Use this to tune tracking performance.
     * @param kP Proportional gain (must be >= 0)
     * @param kD Derivative gain (must be >= 0)
     */
    public void setPIDGains(double kP, double kD) {
        if (kP >= 0 && kD >= 0 && Double.isFinite(kP) && Double.isFinite(kD)) {
            this.kP = kP;
            this.kD = kD;
            resetPID();  // Reset PID state when changing gains
        }
    }

    /**
     * Get current PID gains.
     * @return Array of [kP, kD]
     */
    public double[] getPIDGains() {
        return new double[]{kP, kD};
    }

    // ==================== GETTERS ====================

    public boolean isLocked() { return isLocked; }
    public boolean isTagVisible() { return tagVisible; }
    public double getServoPosition() { return servoPos; }
    public double getTx() { return currentTX; }
    public double getError() { return currentError; }
    public String getDebugState() { return debugState; }
    public int getTargetTagId() { return targetTagId; }
    public String getDetectedTagIds() { return detectedTagIds; }
    public int getDetectedTagCount() { return detectedTagCount; }
    public double getPidOutput() { return pidOutput; }

    /**
     * Check if the turret is actively tracking (target visible and filter initialized).
     * @return true if currently tracking a target
     */
    public boolean isActivelyTracking() {
        return tagVisible && filterInitialized;
    }

    // Compatibility
    public boolean isAligned() { return isLocked; }
    public boolean isTagDetected() { return tagVisible; }
    public double getTy() { return currentTY; }
    public double getTargetArea() { return 0; }
    public double getTxVelocity() { return 0; }
    public double getPredictedTx() { return currentTX; }
    public double getFilteredTx() { return currentTX; }
    public double getConfidence() { return tagVisible ? 1.0 : 0.0; }
    public boolean isResultValid() { return tagVisible; }
    public boolean isLimelightConnected() { return limelight != null && limelight.isConnected(); }
    public double getLastMovement() { return pidOutput; }
    public double getPositionComponent() { return currentError * kP; }
    public double getVelocityComponent() { return 0; }

    public String getStateName() {
        if (isLocked) return "LOCKED";
        if (tagVisible) return "TRACKING";
        return "SCANNING";
    }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        telemetry.addLine("=== AUTO-AIM TURRET (Continuous Track) ===");
        telemetry.addData("Status", debugState);
        telemetry.addData("Drive Ready", driveInitialized ? "YES" : "NO");
        telemetry.addLine("");

        telemetry.addData("TX (raw)", "%.2f°", currentTX);
        telemetry.addData("TX Offset Applied", "%.1f°", LIMELIGHT_TX_OFFSET_DEGREES);
        telemetry.addData("TY", "%.2f°", currentTY);
        telemetry.addData("Error", "%.2f°", currentError);
        telemetry.addData("PID Output", "%.4f", pidOutput);
        telemetry.addLine("");

        telemetry.addData("Servo Position", "%.3f", servoPos);
        telemetry.addData("Locked", isLocked ? "YES" : "no");
        telemetry.addData("Lock Tolerance", "%.1f°", RobotConstants.TURRET_POSITION_TOLERANCE);
        telemetry.addLine("");

        telemetry.addData("Tags Detected", detectedTagCount);
        telemetry.addData("Tag IDs", detectedTagIds);
        telemetry.addData("Target Tag", targetTagId);
        telemetry.addLine("");

        telemetry.addLine("--- PID Tuning ---");
        telemetry.addData("kP", "%.4f", kP);
        telemetry.addData("kD", "%.4f", kD);
        telemetry.addData("Filter", "%.2f", RobotConstants.TURRET_TX_FILTER_ALPHA);
    }
}

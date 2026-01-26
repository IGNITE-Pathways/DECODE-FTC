package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

import java.util.List;

/**
 * Auto-aiming turret using PID control with servo
 * Based on motor PID logic but adapted for servo position control
 */
public class ActualTurretLockOn {

    // Hardware
    private Servo turretServo;
    private Limelight3A limelight;
    private Telemetry telemetry;

    // PID Controller variables - TUNED TO PREVENT OVERSHOOT/HUNTING
    private double kP = 0.008;    // Proportional gain - REDUCED to prevent overshoot
    private double kI = 0.0;      // Integral gain - DISABLED to prevent oscillation
    private double kD = 0.005;    // Derivative gain - INCREASED for damping

    private double targetX = 0.0; // Target is centered (tx = 0)
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    // Deadband and limits - TUNED TO PREVENT HUNTING
    private static final double POSITION_TOLERANCE = 2.5;  // degrees - WIDER to prevent micro-adjustments
    private static final double MIN_ADJUSTMENT = 0.0002;   // Minimum servo adjustment
    private static final double MAX_ADJUSTMENT = 0.012;    // Maximum servo speed - REDUCED to prevent overshoot
    private static final double TARGET_LOST_TIMEOUT = 0.5; // seconds before resetting
    private static final double LOCKED_MICRO_DEADZONE = 0.0001; // Extra deadzone when locked

    // Servo limits
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.9;
    private static final double SERVO_CENTER = 0.5;

    // Direction tuning - CHANGE THIS IF TURRET MOVES WRONG WAY
    private static final boolean INVERT_DIRECTION = false;

    // Target tag
    private int targetTagId = 20;  // Blue = 20, Red = 24
    private static final int APRILTAG_PIPELINE = 3;
    private static final int BLUE_TAG = 20;
    private static final int RED_TAG = 24;

    // Exposure settings
    private boolean autoExposureEnabled = true;
    private int manualExposure = 50;  // 0-100, used when auto is disabled
    private static final int MIN_EXPOSURE = 0;
    private static final int MAX_EXPOSURE = 100;

    // State
    private double servoPos = SERVO_CENTER;
    private boolean targetWasVisible = false;
    private boolean isLocked = false;

    // Debug
    private String debugState = "INIT";

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
                limelight.pipelineSwitch(APRILTAG_PIPELINE); // Pipeline 3 for AprilTag
                limelight.setPollRateHz(100);

                // Configure exposure settings
                configureExposure();

                limelight.start();
            }

            timer.reset();
            targetLostTimer.reset();

            debugState = "Initialized";
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Direction Inverted", INVERT_DIRECTION);
            telemetry.update();
        } catch (Exception e) {
            debugState = "Init Error: " + e.getMessage();
            telemetry.addData("Init Error", e.getMessage());
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

                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        if (fr.getFiducialId() == targetTagId) {
                            targetFiducial = fr;
                            break;
                        }
                    }

                    if (targetFiducial != null) {
                        // TARGET FOUND!
                        targetWasVisible = true;
                        targetLostTimer.reset();

                        // Get horizontal offset from center (tx)
                        double tx = targetFiducial.getTargetXDegrees();

                        // Calculate time since last update
                        double dt = timer.seconds();
                        timer.reset();

                        // Prevent division by zero or huge derivatives
                        if (dt < 0.001) dt = 0.001;
                        if (dt > 1.0) dt = 1.0; // Cap dt if loop was slow

                        // Calculate error
                        double error = tx - targetX;

                        // PID calculations
                        integral += error * dt;

                        // Anti-windup: prevent integral from getting too large
                        integral = Math.max(-50, Math.min(50, integral));

                        double derivative = (error - lastError) / dt;

                        // Calculate PID output
                        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

                        // Apply deadband - stop if close enough
                        if (Math.abs(error) < POSITION_TOLERANCE) {
                            isLocked = true;
                            debugState = "LOCKED ON";

                            // CRITICAL FIX: Zero out PID when locked to prevent overshoot
                            pidOutput = 0;
                            integral = 0;
                            lastError = 0;
                        } else {
                            isLocked = false;
                            debugState = "TRACKING tx=" + String.format("%.1f", tx);

                            // Apply minimum adjustment if not zero (overcome friction)
                            if (pidOutput != 0) {
                                if (Math.abs(pidOutput) < MIN_ADJUSTMENT) {
                                    pidOutput = MIN_ADJUSTMENT * Math.signum(pidOutput);
                                }
                            }
                        }

                        // Clamp output to max adjustment
                        double servoAdjust = Math.max(-MAX_ADJUSTMENT, Math.min(MAX_ADJUSTMENT, pidOutput));

                        // CRITICAL FIX: Micro-deadzone when locked to prevent jitter-induced hunting
                        if (isLocked && Math.abs(servoAdjust) < LOCKED_MICRO_DEADZONE) {
                            servoAdjust = 0;
                        }

                        // Apply inversion if needed
                        if (INVERT_DIRECTION) {
                            servoAdjust = -servoAdjust;
                        }

                        // Safety check: ensure adjustment is finite
                        if (!Double.isFinite(servoAdjust)) {
                            servoAdjust = 0;
                            resetPID();
                        }

                        // Apply to servo position
                        servoPos += servoAdjust;
                        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));

                        lastError = error;

                    } else {
                        // Tags detected but not our target tag
                        handleNoTarget();
                    }
                } else {
                    // Result valid but no fiducials detected
                    handleNoTarget();
                }
            } else {
                // No valid result
                handleNoTarget();
            }

            // Apply servo position with safety check
            if (turretServo != null && Double.isFinite(servoPos)) {
                double clampedPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(clampedPos);
            }

            return true;

        } catch (Exception e) {
            // Catch any errors to prevent crash
            debugState = "Error: " + e.getMessage();
            stopTurret();
            return false;
        }
    }

    private void handleNoTarget() {
        // Target lost - decide what to do
        if (targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
            // Recently lost target, hold position (coast)
            debugState = "TRACKING LOST - Holding (lost " + String.format("%.1fs", targetLostTimer.seconds()) + ")";
            isLocked = false;
        } else {
            // Target lost for too long - stop and reset
            stopTurret();
            debugState = "SEARCHING";
        }
    }

    private void stopTurret() {
        resetPID();
        targetWasVisible = false;
        isLocked = false;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
    }

    /**
     * Configure limelight exposure settings.
     * Called during initialization and when exposure settings change.
     */
    private void configureExposure() {
        if (limelight == null) return;

        try {
            if (autoExposureEnabled) {
                // Enable automatic exposure adjustment
                // The Limelight will automatically adjust exposure for optimal AprilTag detection
                limelight.updatePythonInputs(new double[]{0, 0, 0, 0, 0, 0, 0, 1});
                // Last parameter (1) enables auto-exposure
            } else {
                // Set manual exposure value
                // Value range 0-100 (0 = darkest, 100 = brightest)
                limelight.updatePythonInputs(new double[]{0, 0, 0, 0, 0, 0, manualExposure, 0});
                // Second-to-last parameter is exposure value, last (0) disables auto
            }
        } catch (Exception e) {
            // Exposure control failed, continue anyway
            debugState = "Exposure config failed: " + e.getMessage();
        }
    }

    // ==================== PUBLIC METHODS ====================

    public void stop() {
        try {
            if (limelight != null) {
                limelight.stop();
            }
            resetPID();
            targetWasVisible = false;
            isLocked = false;
        } catch (Exception e) {
            // Fail silently
        }
    }

    public void resetLock() {
        resetPID();
        servoPos = SERVO_CENTER;
        targetWasVisible = false;
        isLocked = false;
        timer.reset();
        targetLostTimer.reset();
    }

    public void setAlliance(AllianceColor alliance) {
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;
        resetPID();
    }

    public void setPositionDirect(double position) {
        if (!Double.isFinite(position)) {
            return;
        }

        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }

        resetPID();
    }

    /**
     * Set PID gains for tuning.
     */
    public void setPIDGains(double kP, double kI, double kD) {
        if (kP >= 0 && kI >= 0 && kD >= 0 &&
            Double.isFinite(kP) && Double.isFinite(kI) && Double.isFinite(kD)) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            resetPID();
        }
    }

    /**
     * Enable or disable automatic exposure adjustment.
     * When enabled, the Limelight automatically adjusts exposure for optimal AprilTag detection.
     * @param enable true to enable auto-exposure, false for manual control
     */
    public void setAutoExposure(boolean enable) {
        this.autoExposureEnabled = enable;
        configureExposure();
    }

    /**
     * Set manual exposure value (only used when auto-exposure is disabled).
     * @param exposure Exposure value (0-100, where 0 = darkest, 100 = brightest)
     */
    public void setManualExposure(int exposure) {
        this.manualExposure = Math.max(MIN_EXPOSURE, Math.min(MAX_EXPOSURE, exposure));
        if (!autoExposureEnabled) {
            configureExposure();
        }
    }

    /**
     * Increase manual exposure by a step (used for tuning).
     * @param step Amount to increase (typically 5 or 10)
     */
    public void increaseExposure(int step) {
        setManualExposure(manualExposure + step);
    }

    /**
     * Decrease manual exposure by a step (used for tuning).
     * @param step Amount to decrease (typically 5 or 10)
     */
    public void decreaseExposure(int step) {
        setManualExposure(manualExposure - step);
    }

    // ==================== GETTERS ====================

    public boolean isLocked() { return isLocked; }
    public double getServoPosition() { return servoPos; }
    public String getDebugState() { return debugState; }
    public int getTargetTagId() { return targetTagId; }
    public boolean isLimelightConnected() {
        return limelight != null && limelight.isConnected();
    }
    public boolean isAutoExposureEnabled() { return autoExposureEnabled; }
    public int getManualExposure() { return manualExposure; }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== AUTO-AIM TURRET (PID) ===");
        telemetry.addData("Status", debugState);
        telemetry.addData("Locked", isLocked ? "YES" : "no");
        telemetry.addData("Servo Position", "%.3f", servoPos);
        telemetry.addData("Target Tag", targetTagId);
        telemetry.addData("Limelight", isLimelightConnected() ? "CONNECTED" : "NOT CONNECTED");
        telemetry.addLine("");

        telemetry.addLine("--- Camera Exposure ---");
        telemetry.addData("Mode", autoExposureEnabled ? "AUTO" : "MANUAL");
        if (!autoExposureEnabled) {
            telemetry.addData("Manual Value", manualExposure);
        }
        telemetry.addLine("");

        telemetry.addLine("--- PID Gains ---");
        telemetry.addData("kP", "%.4f", kP);
        telemetry.addData("kI", "%.6f", kI);
        telemetry.addData("kD", "%.4f", kD);
    }
}

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
 * Auto-aiming turret for servo - based on AutoAimingTurret pattern
 */
public class TurretLockOptimized {

    // Hardware
    private Servo turretServo;
    private Limelight3A limelight;
    private Telemetry telemetry;

    // PID Controller variables - TUNE THESE
    private double kP = 0.0005;    // Proportional gain - START LOW and increase
    private double kI = 0.0001;   // Integral gain - usually keep small
    private double kD = 0.002;    // Derivative gain - helps reduce overshoot

    private double targetX = 0.0; // Target is centered (tx = 0)
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    // Deadband and limits
    private static final double POSITION_TOLERANCE = 1.0; // degrees
    private static final double MIN_ADJUSTMENT = 0.001;   // Minimum servo move to overcome deadzone
    private static final double MAX_ADJUSTMENT = 0.03;    // Maximum servo move per cycle
    private static final double TARGET_LOST_TIMEOUT = 0.5; // seconds before scanning

    // Servo limits
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 0.5;
    private static final double SERVO_CENTER = 0.0;

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
                        tagVisible = true;
                        targetWasVisible = true;
                        targetLostTimer.reset();

                        // Get horizontal offset from center (tx)
                        double tx = targetFiducial.getTargetXDegrees();
                        currentTX = tx;

                        // Calculate time since last update
                        double dt = timer.seconds();
                        timer.reset();

                        // Prevent division by zero or huge derivatives
                        if (dt < 0.001) dt = 0.001;
                        if (dt > 1.0) dt = 1.0;

                        // Calculate error
                        double error = tx - targetX;
                        currentError = error;

                        // PID calculations
                        integral += error * dt;

                        // Anti-windup: prevent integral from getting too large
                        integral = Math.max(-50, Math.min(50, integral));

                        double derivative = (error - lastError) / dt;

                        // Calculate PID output
                        pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

                        // Apply deadband - stop if close enough
                        if (Math.abs(error) < POSITION_TOLERANCE) {
                            pidOutput = 0;
                            integral = 0; // Reset integral when on target
                            isLocked = true;
                            debugState = "LOCKED ON";
                        } else {
                            isLocked = false;
                            debugState = "TRACKING";

                            // Apply minimum adjustment if not zero (overcome servo deadzone)
                            if (pidOutput != 0) {
                                if (Math.abs(pidOutput) < MIN_ADJUSTMENT) {
                                    pidOutput = MIN_ADJUSTMENT * Math.signum(pidOutput);
                                }
                            }
                        }

                        // Clamp output to max adjustment
                        double servoAdjust = Math.max(-MAX_ADJUSTMENT, Math.min(MAX_ADJUSTMENT, pidOutput));

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

                        lastError = error;

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

            // Apply servo position
            if (turretServo != null) {
                turretServo.setPosition(servoPos);
            }

            return true;

        } catch (Exception e) {
            debugState = "Error: " + e.getMessage();
            return false;
        }
    }

    private void handleNoTarget() {
        tagVisible = false;

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
    }

    // ==================== PUBLIC METHODS ====================

    public void stop() {
        try {
            if (limelight != null) {
                limelight.stop();
            }
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
    }

    public void setPositionDirect(double position) {
        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }
    }

    public void lock() { isLocked = true; }
    public void unlock() { isLocked = false; }

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

    // Compatibility
    public boolean isAligned() { return isLocked; }
    public boolean isTagDetected() { return tagVisible; }
    public double getTy() { return 0; }
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
        telemetry.addLine("=== AUTO-AIM TURRET ===");
        telemetry.addData("Status", debugState);
        telemetry.addLine("");

        telemetry.addData("TX (degrees)", "%.2f", currentTX);
        telemetry.addData("Target Position", currentTX > 0 ? "RIGHT" : (currentTX < 0 ? "LEFT" : "CENTER"));
        telemetry.addData("Error", "%.2f", currentError);
        telemetry.addData("PID Output", "%.4f", pidOutput);
        telemetry.addLine("");

        telemetry.addData("Servo Position", "%.3f", servoPos);
        telemetry.addData("Locked", isLocked ? "YES" : "no");
        telemetry.addLine("");

        telemetry.addData("Tags Detected", detectedTagCount);
        telemetry.addData("Tag IDs", detectedTagIds);
        telemetry.addData("Looking For", "Tag " + targetTagId);
        telemetry.addLine("");

        telemetry.addLine("--- PID Tuning ---");
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("Inverted", INVERT_DIRECTION);
    }
}

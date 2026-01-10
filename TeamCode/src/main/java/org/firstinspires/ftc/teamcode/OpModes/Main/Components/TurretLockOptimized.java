package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

import java.util.List;

/**
 * SIMPLIFIED Turret Lock - Easy to tune!
 *
 * ============== HOW IT WORKS ==============
 * 1. If AprilTag visible: Move servo to center the tag
 * 2. If no tag: Slowly scan back and forth
 * 3. If centered (within DEADBAND): Stop moving, you're locked!
 *
 * ============== TUNING GUIDE ==============
 * Only 4 values to tune (all at the top):
 *
 * KP (0.002 default):
 *   - Turret too slow? INCREASE this
 *   - Turret overshoots/oscillates? DECREASE this
 *
 * DEADBAND (2.0 default):
 *   - Won't lock on? INCREASE this (try 3.0 or 4.0)
 *   - Locks but not centered? DECREASE this
 *
 * MAX_SPEED (0.008 default):
 *   - Turret too jerky? DECREASE this
 *   - Turret too slow to catch up? INCREASE this
 *
 * SCAN_SPEED (0.002 default):
 *   - Scanning too slow? INCREASE this
 *   - Scanning too fast? DECREASE this
 */
public class TurretLockOptimized {

    // ==================== TUNE THESE VALUES ====================

    // How fast turret responds to error (higher = faster but may oscillate)
    private static final double KP = 0.002;

    // How close to center before we consider "locked" (degrees)
    private static final double DEADBAND = 2.0;

    // Maximum servo movement per loop (limits speed)
    private static final double MAX_SPEED = 0.008;

    // How fast to scan when searching for tag
    private static final double SCAN_SPEED = 0.002;

    // ==================== SERVO LIMITS (don't change) ====================
    private static final double SERVO_MIN = 0.375;
    private static final double SERVO_MAX = 0.8;
    private static final double SERVO_CENTER = 0.5;

    // ==================== APRILTAG IDS ====================
    private static final int BLUE_TAG = 20;
    private static final int RED_TAG = 24;

    // ==================== HARDWARE ====================
    private Limelight3A limelight;
    private Servo turretServo;
    private Telemetry telemetry;

    // ==================== STATE ====================
    private double servoPos = SERVO_CENTER;
    private int targetTagId = BLUE_TAG;
    private boolean tagVisible = false;
    private boolean isLocked = false;
    private boolean isManualLock = false;
    private double manualLockPos = SERVO_CENTER;

    // Scanning
    private int scanDirection = 1;

    // Vision data (for telemetry)
    private double lastTx = 0;
    private double lastTa = 0;

    // ==================== INITIALIZATION ====================

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        initialize(hardwareMap, telemetry, AllianceColor.BLUE);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance) {
        this.telemetry = telemetry;
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }

        // Initialize servo
        turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }

        telemetry.addLine("TurretLockOptimized Ready");
        telemetry.addData("Target Tag", targetTagId);
    }

    // ==================== MAIN UPDATE ====================

    public boolean update() {
        // Manual lock mode - just hold position
        if (isManualLock) {
            servoPos = manualLockPos;
            if (turretServo != null) {
                turretServo.setPosition(servoPos);
            }
            return true;
        }

        // Check hardware
        if (limelight == null || !limelight.isConnected()) {
            return false;
        }

        // Get vision data
        double tx = 0;
        tagVisible = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                if (fr.getFiducialId() == targetTagId) {
                    tx = fr.getTargetXDegrees();
                    lastTa = result.getTa();
                    tagVisible = true;
                    break;
                }
            }
        }

        // Calculate servo movement
        double movement = 0;

        if (tagVisible) {
            lastTx = tx;

            // Check if we're locked (within deadband)
            if (Math.abs(tx) <= DEADBAND) {
                isLocked = true;
                movement = 0;  // Don't move, we're centered!
            } else {
                isLocked = false;
                // Simple proportional control: move toward center
                movement = KP * tx;
            }
        } else {
            // No tag visible - scan to find it
            isLocked = false;
            movement = scanDirection * SCAN_SPEED;

            // Reverse at limits
            if (servoPos >= SERVO_MAX - 0.02) {
                scanDirection = -1;
            } else if (servoPos <= SERVO_MIN + 0.02) {
                scanDirection = 1;
            }
        }

        // Limit speed
        movement = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, movement));

        // Apply movement
        servoPos += movement;
        servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));

        // Set servo
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }

        return true;
    }

    // ==================== PUBLIC METHODS ====================

    public void lock() {
        isManualLock = true;
        manualLockPos = servoPos;
    }

    public void unlock() {
        isManualLock = false;
    }

    public void setPositionDirect(double position) {
        position = Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
        manualLockPos = position;
        servoPos = position;
        isManualLock = true;
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }
    }

    public void setAlliance(AllianceColor alliance) {
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_TAG : RED_TAG;
    }

    public void resetLock() {
        isManualLock = false;
        isLocked = false;
        servoPos = SERVO_CENTER;
    }

    // ==================== GETTERS ====================

    public boolean isLocked() {
        return isLocked;
    }

    public boolean isTagVisible() {
        return tagVisible;
    }

    public double getServoPosition() {
        return servoPos;
    }

    public double getTx() {
        return lastTx;
    }

    public double getTargetArea() {
        return lastTa;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    public String getStateName() {
        if (isManualLock) return "MANUAL";
        if (isLocked) return "LOCKED";
        if (tagVisible) return "TRACKING";
        return "SCANNING";
    }

    // For compatibility
    public boolean isAligned() { return isLocked; }
    public boolean isTagDetected() { return tagVisible; }
    public boolean isInLockMode() { return isLocked || isManualLock; }
    public double getFilteredTx() { return lastTx; }
    public double getCompensatedTx() { return lastTx; }
    public double getConfidence() { return tagVisible ? 1.0 : 0.0; }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("State", getStateName());
        telemetry.addData("Tag", tagVisible ? "VISIBLE" : "NOT FOUND");
        telemetry.addData("tx", "%.1f°", lastTx);
        telemetry.addData("Servo", "%.3f", servoPos);
        telemetry.addData("Target", "Tag " + targetTagId);
        telemetry.addLine("");
        telemetry.addLine("Tune: KP=" + KP + " DEADBAND=" + DEADBAND);
    }
}

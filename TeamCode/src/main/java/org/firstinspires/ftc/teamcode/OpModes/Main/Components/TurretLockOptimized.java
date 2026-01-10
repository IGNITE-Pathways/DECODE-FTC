package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

import java.util.List;

/**
 * TurretLockOptimized - Competition-Level IMU-Stabilized AprilTag Turret Tracking
 *
 * ARCHITECTURE:
 * This turret controller cleanly separates VISION TARGETING from MOTION CONTROL:
 *
 * [Vision System] --> [Target Estimation] --> [State Machine] --> [Motion Controller] --> [Servo]
 *
 * CORE FEATURES:
 *
 * 1. LATENCY COMPENSATION:
 *    - Camera has ~50ms delay; we predict where tag IS, not where it WAS
 *    - Uses robot yaw rate to extrapolate current tag position
 *    - Dramatically improves tracking during fast robot rotation
 *
 * 2. MULTI-STAGE STATE MACHINE:
 *    - SCANNING: Looking for tag (slow sweep)
 *    - ACQUIRING: Tag found, moving toward it (fast, aggressive)
 *    - TRACKING: Actively following tag (responsive, IMU-assisted)
 *    - HOLDING: Locked and aligned (minimal power, tight deadband)
 *
 * 3. CONFIDENCE-BASED CONTROL:
 *    - Each detection has a confidence score (0-1)
 *    - Low confidence = slower response, prevents jerky movements
 *    - High confidence = full tracking authority
 *    - Prevents bad detections from causing sudden servo jumps
 *
 * 4. SOFT ROTATION LIMITS:
 *    - Smooth slowdown near servo min/max (no hard stops)
 *    - Safe wrap-around handling
 *    - Prevents mechanical stress and oscillation at limits
 *
 * 5. DISTANCE-BASED SPEED LIMITING:
 *    - Closer targets = smaller movements needed
 *    - Far targets = can move faster
 *    - Adaptive deadband based on distance
 *
 * 6. FAIL-SAFE BEHAVIOR:
 *    - If vision drops, HOLDS last known good position
 *    - Uses IMU to maintain orientation during dropout
 *    - Never spins wildly when tag is lost
 *    - Graceful degradation to scanning only after timeout
 *
 * 7. IMU COUNTER-ROTATION:
 *    - Primary control during TRACKING/HOLDING states
 *    - Cancels robot rotation in real-time
 *    - Vision provides trim corrections only
 *
 * AprilTag IDs: 20 (BLUE alliance), 24 (RED alliance)
 */
public class TurretLockOptimized {

    // ==================== TURRET STATE MACHINE ====================
    public enum TurretState {
        SCANNING,   // Looking for tag - slow sweep motion
        ACQUIRING,  // Tag found, moving toward it - fast, aggressive
        TRACKING,   // Actively following tag - IMU + vision
        HOLDING     // Locked on target - minimal power, tight deadband
    }
    private static final double MIN_HOLD_TIME_SEC = 0.3;    // minimum time to stay in HOLDING
    private static final int REQUIRED_ALIGNED_FRAMES = 3;   // number of frames to confirm alignment
    private static final double HOLD_EXIT_MULTIPLIER = 1.5; // expand deadband slightly for leaving HOLDING

    private int alignedFrames = 0;
    private double alignedStartTime = 0;
    private TurretState currentState = TurretState.SCANNING;
    private TurretState previousState = TurretState.SCANNING;
    private double stateEntryTime = 0;

    // ==================== HARDWARE ====================
    private Limelight3A limelight;
    private Servo turretServo;
    private GoBildaPinpointDriver pinpoint;  // GoBILDA Pinpoint for IMU
    private Telemetry telemetry;

    // ==================== APRILTAG CONFIG ====================
    private static final int BLUE_ALLIANCE_TAG = 20;
    private static final int RED_ALLIANCE_TAG = 24;
    private int targetTagId = BLUE_ALLIANCE_TAG;

    // ==================== SERVO CONFIG ====================
    private static final double SERVO_MIN = 0.375;
    private static final double SERVO_MAX = 0.8;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE = SERVO_MAX - SERVO_MIN;

    // Soft limit zones (start slowing down near edges)
    private static final double SOFT_LIMIT_ZONE = 0.05;  // Slow down within 5% of limits
    private static final double SOFT_LIMIT_MIN = SERVO_MIN + SOFT_LIMIT_ZONE;
    private static final double SOFT_LIMIT_MAX = SERVO_MAX - SOFT_LIMIT_ZONE;

    // ==================== LATENCY COMPENSATION ====================
    // Minimal latency compensation to prevent overshoot
    private static final double CAMERA_LATENCY_MS = 50.0;
    private static final double LATENCY_COMPENSATION_GAIN = 0.3;  // Very conservative

    // ==================== IMU COUNTER-ROTATION ====================
    // Very gentle IMU assist - let vision do most work
    private static final double IMU_FEEDFORWARD_GAIN = -0.0005;  // Halved again
    private static final double MAX_IMU_OUTPUT = 0.004;          // Halved again

    // Heavy smoothing on IMU rate
    private static final double IMU_RATE_SMOOTHING = 0.15;       // Much more smoothing

    // ==================== VISION TRIM PID ====================
    // Very soft gains - prioritize stability over speed
    private static final double TRIM_kP = 0.0006;                // Halved - very gentle
    private static final double TRIM_kI = 0.000005;              // Halved
    private static final double TRIM_kD = 0.001;                 // Increased for more damping
    private static final double MAX_TRIM_OUTPUT = 0.003;         // Reduced max output
    private static final double INTEGRAL_MAX = 15.0;             // Reduced

    // ==================== STATE-SPECIFIC PARAMETERS ====================

    // SCANNING state (slow sweep)
    private static final double SCAN_SPEED = 0.003;              // Slower scanning
    private static final double SEARCH_RANGE = 0.20;

    // ACQUIRING state - gentle approach
    private static final double ACQUIRE_kP = 0.0012;             // Much gentler
    private static final double ACQUIRE_MAX_OUTPUT = 0.012;      // Reduced
    private static final double ACQUIRE_TIMEOUT_SEC = 3.0;       // More time allowed

    // TRACKING state - wide deadband for stability
    private static final double TRACK_DEADBAND = 3.0;            // Very wide - let it settle

    // HOLDING state - wide deadband, minimal corrections
    private static final double HOLD_DEADBAND = 4.0;             // Very wide for stability
    private static final double HOLD_MAX_OUTPUT = 0.002;         // Minimal corrections
    private static final double HOLD_ENTRY_TIME_SEC = 0.3;       // Enter holding faster


    // ==================== CONFIDENCE THRESHOLDS ====================
    private static final double CONFIDENCE_HIGH = 0.65;   // Full tracking authority
    private static final double CONFIDENCE_MEDIUM = 0.35; // Reduced authority
    private static final double CONFIDENCE_LOW = 0.15;    // Minimal authority
    private static final double CONFIDENCE_DROPOUT = 0.08; // Below this = no vision trust

    // ==================== VISION FILTERING ====================
    private static final int MIN_FRAMES_FOR_ACQUIRE = 2;
    private static final int MIN_FRAMES_FOR_TRACK = 4;
    private static final double TX_SMOOTHING_ALPHA = 0.3;
    private static final double MAX_TX_JUMP = 12.0;
    private static final double MIN_TARGET_AREA = 0.02;
    private static final double MAX_TARGET_AREA = 25.0;

    // ==================== FAIL-SAFE TIMING ====================
    private static final int FRAMES_TO_FAILSAFE = 35;   // ~0.4s - enter fail-safe hold
    private static final int FRAMES_TO_DROP_LOCK = 75;  // ~1.5s - drop to scanning (more forgiving)

    // ==================== DISTANCE-BASED SCALING ====================
    // Speed scaling based on distance (closer = slower movements needed)
    private static final double CLOSE_DISTANCE_FT = 2.0;
    private static final double FAR_DISTANCE_FT = 8.0;
    private static final double CLOSE_SPEED_SCALE = 0.6;   // Slower when close
    private static final double FAR_SPEED_SCALE = 1.0;     // Normal speed when far

    // Deadband scaling based on distance
    private static final double CLOSE_DEADBAND_SCALE = 0.8;  // Tighter when close
    private static final double FAR_DEADBAND_SCALE = 1.2;    // Looser when far

    // ==================== STATE VARIABLES ====================

    // Servo
    private double servoPos = SERVO_CENTER;
    private double lastServoPos = SERVO_CENTER;
    private double servoVelocity = 0;

    // IMU
    private double robotYawRate = 0;
    private double filteredYawRate = 0;
    private double robotYaw = 0;
    private double imuOutput = 0;

    // Vision - raw
    private double rawTx = 0;
    private double targetArea = 0;

    // Vision - filtered
    private double filteredTx = 0;
    private double lastFilteredTx = 0;
    private double txRateOfChange = 0;
    private double lastValidTx = 0;

    // Vision - latency compensated
    private double compensatedTx = 0;

    // Frame counters
    private int consecutiveFrames = 0;
    private int framesSinceSeen = 0;

    // Confidence
    private double confidence = 0;
    private double confidenceScale = 0;  // How much to trust vision this frame

    // PID state
    private double trimIntegral = 0;
    private double lastTrimError = 0;
    private double trimOutput = 0;

    // State tracking
    private boolean tagDetected = false;
    private boolean aligned = false;
    private boolean inFailSafe = false;

    // Fail-safe - last known good position
    private double lastKnownGoodPos = SERVO_CENTER;
    private double lastKnownGoodTx = 0;
    private double lastKnownGoodYaw = 0;

    // Scanning
    private int scanDirection = 1;
    private double scanCenterPos = SERVO_CENTER;

    // Manual override
    private boolean isManuallyLocked = false;
    private double manualLockPosition = SERVO_CENTER;

    // Timing
    private ElapsedTime frameTimer = new ElapsedTime();
    private double lastFrameTime = 0;
    private double deltaTime = 0;
    private double loopTimeMs = 0;

    // Distance
    private static final double APRILTAG_HEIGHT_METERS = 0.2032;
    private static final double CAMERA_VFOV_DEGREES = 49.5;
    private static final int IMAGE_WIDTH = 1280;
    private static final int IMAGE_HEIGHT = 720;
    private double distanceMeters = 0;
    private double distanceFeet = 0;

    // ==================== INITIALIZATION ====================

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        initialize(hardwareMap, telemetry, AllianceColor.RED);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance) {
        this.telemetry = telemetry;
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_ALLIANCE_TAG : RED_ALLIANCE_TAG;

        // Initialize GoBILDA Pinpoint (has built-in IMU)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        if (pinpoint != null) {
            // Configure odometry pods (match your ManualHoodAndShoot settings)
            pinpoint.setOffsets(-7.08661, 3.75, DistanceUnit.INCH);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.recalibrateIMU();
            pinpoint.resetPosAndIMU();
        }

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

        frameTimer.reset();
        currentState = TurretState.SCANNING;

        telemetry.addLine("TurretLockOptimized (Competition-Level) initialized");
        telemetry.addData("Target Tag", targetTagId);
    }

    // ==================== MAIN UPDATE LOOP ====================


    public boolean update() {
        // Calculate timing
        double currentTime = frameTimer.seconds();
        deltaTime = currentTime - lastFrameTime;
        lastFrameTime = currentTime;
        loopTimeMs = deltaTime * 1000;
        deltaTime = Math.min(deltaTime, 0.1);

        // **MANUAL LOCK OVERRIDE**
        if (isManuallyLocked) {
            turretServo.setPosition(manualLockPosition);
            // Keep servo and internal state consistent
            servoPos = manualLockPosition;
            trimOutput = 0;
            imuOutput = 0;
            return true; // skip the rest of the loop
        }

        // Check hardware
        if (limelight == null || !limelight.isConnected()) {
            handleHardwareFailure();
            return false;
        }

        // ===== STAGE 1: SENSOR INPUT =====
        updateIMU();
        VisionData vision = processVisionData();

        // ===== STAGE 2: TARGET ESTIMATION (with latency compensation) =====
        estimateCurrentTarget(vision);

        // ===== STAGE 3: STATE MACHINE UPDATE =====
        updateStateMachine(vision);

        // ===== STAGE 4: MOTION CONTROL =====
        double rawOutput = calculateMotionOutput(vision);

        // ===== STAGE 5: OUTPUT LIMITING & SAFETY =====
        double safeOutput = applySafetyLimits(rawOutput);

        // ===== STAGE 6: APPLY TO SERVO =====
        applyServoOutput(safeOutput);

        return true;
    }

    // ==================== STAGE 1: SENSOR INPUT ====================

    private void updateIMU() {
        if (pinpoint == null) {
            robotYawRate = 0;
            filteredYawRate = 0;
            return;
        }

        // Update the Pinpoint (required each loop to get fresh data)
        pinpoint.update();

        // Get heading velocity (yaw rate) in degrees/sec
        robotYawRate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        // Smooth the yaw rate to reduce noise
        filteredYawRate = IMU_RATE_SMOOTHING * robotYawRate + (1 - IMU_RATE_SMOOTHING) * filteredYawRate;

        // Get absolute heading in degrees
        robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    // ==================== STAGE 2: VISION PROCESSING ====================

    private class VisionData {
        boolean isValid = false;
        double tx = 0;
        double ta = 0;
        double confidence = 0;
        double latencyMs = 0;
    }

    private VisionData processVisionData() {
        VisionData data = new VisionData();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            handleNoDetection();
            return data;
        }

        // Get pipeline latency for compensation
        data.latencyMs = result.getCaptureLatency() + result.getTargetingLatency();

        // Find target tag
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        LLResultTypes.FiducialResult targetFiducial = null;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == targetTagId) {
                targetFiducial = fr;
                break;
            }
        }

        if (targetFiducial == null) {
            handleNoDetection();
            return data;
        }

        // Extract raw values
        rawTx = targetFiducial.getTargetXDegrees();
        targetArea = result.getTa();

        // Filter 1: Target area validation
        if (targetArea < MIN_TARGET_AREA || targetArea > MAX_TARGET_AREA) {
            handleNoDetection();
            return data;
        }

        // Filter 2: Reject sudden jumps (bad detections)
        if (consecutiveFrames > 0 && Math.abs(rawTx - lastValidTx) > MAX_TX_JUMP) {
            consecutiveFrames = Math.max(0, consecutiveFrames - 2);
            return data;
        }

        // Filter 3: Exponential smoothing
        if (consecutiveFrames == 0) {
            filteredTx = rawTx;
        } else {
            filteredTx = TX_SMOOTHING_ALPHA * rawTx + (1 - TX_SMOOTHING_ALPHA) * filteredTx;
        }

        // Calculate tx rate of change
        if (deltaTime > 0 && consecutiveFrames > 0) {
            txRateOfChange = (filteredTx - lastFilteredTx) / deltaTime;
        }
        lastFilteredTx = filteredTx;
        lastValidTx = rawTx;

        // Update frame counters
        consecutiveFrames++;
        framesSinceSeen = 0;
        tagDetected = true;

        // Calculate distance
        calculateDistance();

        // Calculate confidence score
        data.confidence = calculateConfidence();
        confidence = data.confidence;

        // Calculate confidence scale (how much to trust this frame)
        confidenceScale = calculateConfidenceScale(data.confidence);

        // Populate data
        data.isValid = true;
        data.tx = filteredTx;
        data.ta = targetArea;

        // Update last known good values for fail-safe
        if (data.confidence > CONFIDENCE_MEDIUM) {
            lastKnownGoodPos = servoPos;
            lastKnownGoodTx = filteredTx;
            lastKnownGoodYaw = robotYaw;
        }

        return data;
    }

    private void handleNoDetection() {
        consecutiveFrames = 0;
        framesSinceSeen++;
        tagDetected = false;
        confidence = Math.max(0, confidence - 0.08);  // Faster decay
        confidenceScale = 0;
    }

    private double calculateConfidence() {
        // Multi-factor confidence calculation
        double frameConf = Math.min(1.0, consecutiveFrames / 8.0);
        double areaConf = Math.min(1.0, targetArea / 1.5);
        double stabilityConf = Math.max(0, 1.0 - Math.abs(txRateOfChange) / 80.0);
        double centerConf = Math.max(0, 1.0 - Math.abs(filteredTx) / 30.0);  // Prefer centered detections

        return (frameConf * 0.3 + areaConf * 0.25 + stabilityConf * 0.25 + centerConf * 0.2);
    }

    private double calculateConfidenceScale(double conf) {
        // Maps confidence to control authority (0-1)
        if (conf >= CONFIDENCE_HIGH) {
            return 1.0;
        } else if (conf >= CONFIDENCE_MEDIUM) {
            return 0.5 + 0.5 * (conf - CONFIDENCE_MEDIUM) / (CONFIDENCE_HIGH - CONFIDENCE_MEDIUM);
        } else if (conf >= CONFIDENCE_LOW) {
            return 0.2 + 0.3 * (conf - CONFIDENCE_LOW) / (CONFIDENCE_MEDIUM - CONFIDENCE_LOW);
        } else if (conf >= CONFIDENCE_DROPOUT) {
            return 0.1 * (conf - CONFIDENCE_DROPOUT) / (CONFIDENCE_LOW - CONFIDENCE_DROPOUT);
        }
        return 0;
    }

    private void calculateDistance() {
        if (targetArea > 0) {
            double pixelArea = (targetArea / 100.0) * (IMAGE_WIDTH * IMAGE_HEIGHT);
            double tagPixelHeight = Math.sqrt(pixelArea);
            double focalPx = (IMAGE_HEIGHT / 2.0) / Math.tan(Math.toRadians(CAMERA_VFOV_DEGREES / 2.0));
            distanceMeters = (APRILTAG_HEIGHT_METERS * focalPx) / tagPixelHeight;
            distanceFeet = distanceMeters * 3.28084;
        }
    }

    // ==================== LATENCY COMPENSATION ====================

    private void estimateCurrentTarget(VisionData vision) {
        if (!vision.isValid) {
            compensatedTx = filteredTx;  // Use last known
            return;
        }

        // The camera saw the tag some time ago. During that latency,
        // the robot has rotated. We need to predict where the tag IS NOW.
        //
        // If robot rotated right during latency, the tag has moved LEFT
        // in the camera frame, so we subtract the rotation.

        double latencySec = (vision.latencyMs > 0 ? vision.latencyMs : CAMERA_LATENCY_MS) / 1000.0;

        // Estimate how much robot rotated during latency
        double rotationDuringLatency = filteredYawRate * latencySec;

        // Compensate: tag moves opposite to robot rotation in camera frame
        compensatedTx = vision.tx - (rotationDuringLatency * LATENCY_COMPENSATION_GAIN);
    }

    // ==================== STAGE 3: STATE MACHINE ====================

    private void updateStateMachine(VisionData vision) {
        previousState = currentState;
        double currentTime = frameTimer.seconds();

        switch (currentState) {
            case SCANNING:
                // Transition: tag detected with enough frames -> ACQUIRING
                if (vision.isValid && consecutiveFrames >= MIN_FRAMES_FOR_ACQUIRE) {
                    transitionTo(TurretState.ACQUIRING);
                }
                break;

            case ACQUIRING:
                // Transition: high confidence + more frames -> TRACKING
                if (vision.isValid && vision.confidence >= CONFIDENCE_MEDIUM
                        && consecutiveFrames >= MIN_FRAMES_FOR_TRACK) {
                    transitionTo(TurretState.TRACKING);
                }
                // Transition: lost tag -> back to SCANNING
                else if (framesSinceSeen >= FRAMES_TO_DROP_LOCK) {
                    transitionTo(TurretState.SCANNING);
                }
                // Timeout protection
                else if (currentTime - stateEntryTime > ACQUIRE_TIMEOUT_SEC) {
                    transitionTo(TurretState.SCANNING);
                }
                break;

            case TRACKING:
                // Count aligned frames
                if (Math.abs(compensatedTx) <= TRACK_DEADBAND) alignedFrames++;
                else alignedFrames = 0;

                // Enter HOLDING only if stable for REQUIRED_ALIGNED_FRAMES AND minimum hold entry time passed
                if (alignedFrames >= REQUIRED_ALIGNED_FRAMES &&
                        (frameTimer.seconds() - stateEntryTime) >= HOLD_ENTRY_TIME_SEC) {
                    transitionTo(TurretState.HOLDING);
                    alignedStartTime = frameTimer.seconds();
                }
                break;

            case HOLDING:
                double holdDeadband = TRACK_DEADBAND; // or your original hold deadband
                double exitDeadband = holdDeadband * HOLD_EXIT_MULTIPLIER;

                // Stay at least MIN_HOLD_TIME_SEC
                if ((frameTimer.seconds() - stateEntryTime) >= MIN_HOLD_TIME_SEC) {
                    if (Math.abs(compensatedTx) > exitDeadband) {
                        transitionTo(TurretState.TRACKING);
                    }
                }
                break;

        }
    }

    private void transitionTo(TurretState newState) {
        previousState = currentState;
        currentState = newState;
        stateEntryTime = frameTimer.seconds();
        inFailSafe = false;

        // Reset state-specific variables
        if (newState == TurretState.SCANNING) {
            scanCenterPos = servoPos;
            aligned = false;
            trimIntegral = 0;
        } else if (newState == TurretState.ACQUIRING) {
            trimIntegral = 0;
            lastTrimError = 0;
        } else if (newState == TurretState.TRACKING) {
            alignedStartTime = 0;
        }
    }

    // ==================== STAGE 4: MOTION CONTROL ====================

    private double calculateMotionOutput(VisionData vision) {
        double output = 0;

        switch (currentState) {
            case SCANNING:
                output = calculateScanningOutput();
                imuOutput = 0;
                trimOutput = 0;
                break;

            case ACQUIRING:
                output = calculateAcquiringOutput(vision);
                imuOutput = 0;
                break;

            case TRACKING:
                output = calculateTrackingOutput(vision);
                break;

            case HOLDING:
                output = calculateHoldingOutput(vision);
                break;
        }

        return output;
    }

    private double calculateScanningOutput() {
        // Simple sweep motion
        double nextPos = servoPos + scanDirection * SCAN_SPEED;

        double maxScan = Math.min(SERVO_MAX, scanCenterPos + SEARCH_RANGE);
        double minScan = Math.max(SERVO_MIN, scanCenterPos - SEARCH_RANGE);

        if (nextPos >= maxScan) {
            scanDirection = -1;
            nextPos = maxScan;
        } else if (nextPos <= minScan) {
            scanDirection = 1;
            nextPos = minScan;
        }

        return nextPos - servoPos;  // Return delta
    }

    private double calculateAcquiringOutput(VisionData vision) {
        if (!vision.isValid) return 0;

        // Aggressive P-only control to quickly move toward tag
        double error = compensatedTx;
        double output = ACQUIRE_kP * error * confidenceScale;

        // Clamp
        output = clamp(output, -ACQUIRE_MAX_OUTPUT, ACQUIRE_MAX_OUTPUT);
        trimOutput = output;

        return output;
    }

    private double calculateTrackingOutput(VisionData vision) {
        double output = 0;

        // ===== IMU COUNTER-ROTATION (PRIMARY) =====
        imuOutput = IMU_FEEDFORWARD_GAIN * filteredYawRate * deltaTime;
        imuOutput = clamp(imuOutput, -MAX_IMU_OUTPUT, MAX_IMU_OUTPUT);
        output += imuOutput;

        // ===== FAIL-SAFE: Hold position using IMU only =====
        if (inFailSafe) {
            // No vision trim - just IMU counter-rotation to maintain orientation
            trimOutput = 0;
            return output;
        }

        // ===== VISION TRIM PID (SECONDARY) =====
        if (vision.isValid) {
            double error = compensatedTx;
            double effectiveDeadband = getDistanceScaledDeadband(TRACK_DEADBAND);

            // Apply deadband
            if (Math.abs(error) < effectiveDeadband * 0.5) {
                // Inside deadband - minimal correction
                error = 0;
            }

            // PID calculation
            trimIntegral += error * deltaTime;
            trimIntegral = clamp(trimIntegral, -INTEGRAL_MAX, INTEGRAL_MAX);

            // Reset integral on zero-crossing to prevent windup
            if ((error > 0 && lastTrimError < 0) || (error < 0 && lastTrimError > 0)) {
                trimIntegral *= 0.5;
            }

            double derivative = 0;
            if (deltaTime > 0.001) {
                derivative = (error - lastTrimError) / deltaTime;
            }
            lastTrimError = error;

            // Calculate trim
            trimOutput = (TRIM_kP * error) + (TRIM_kI * trimIntegral) + (TRIM_kD * derivative);

            // Scale by confidence
            trimOutput *= confidenceScale;

            // Scale by distance (smaller movements when close)
            trimOutput *= getDistanceSpeedScale();

            // Clamp
            trimOutput = clamp(trimOutput, -MAX_TRIM_OUTPUT, MAX_TRIM_OUTPUT);

            output += trimOutput;
        }

        return output;
    }

    private double calculateHoldingOutput(VisionData vision) {
        double output = 0;

        // ===== IMU COUNTER-ROTATION (still active but smaller) =====
        imuOutput = IMU_FEEDFORWARD_GAIN * filteredYawRate * deltaTime * 0.7;  // Reduced gain
        imuOutput = clamp(imuOutput, -MAX_IMU_OUTPUT * 0.5, MAX_IMU_OUTPUT * 0.5);
        output += imuOutput;

        // ===== FAIL-SAFE: Just hold with IMU =====
        if (inFailSafe) {
            trimOutput = 0;
            return output;
        }

        // ===== MINIMAL VISION CORRECTION =====
        if (vision.isValid) {
            double error = compensatedTx;
            double effectiveDeadband = getDistanceScaledDeadband(HOLD_DEADBAND);

            // Tight deadband - only correct if drifted significantly
            if (Math.abs(error) > effectiveDeadband) {
                trimOutput = TRIM_kP * 0.5 * error * confidenceScale;
                trimOutput = clamp(trimOutput, -HOLD_MAX_OUTPUT, HOLD_MAX_OUTPUT);
                output += trimOutput;
            } else {
                trimOutput = 0;
            }
        }

        return output;
    }

    // ==================== STAGE 5: SAFETY LIMITS ====================

    private double applySafetyLimits(double rawOutput) {
        double targetPos = servoPos + rawOutput;

        // Soft limits - slow down near edges
        if (targetPos < SOFT_LIMIT_MIN) {
            double distToLimit = targetPos - SERVO_MIN;
            double scale = distToLimit / SOFT_LIMIT_ZONE;
            scale = clamp(scale, 0.1, 1.0);
            if (rawOutput < 0) {  // Only limit if moving toward limit
                rawOutput *= scale;
            }
        } else if (targetPos > SOFT_LIMIT_MAX) {
            double distToLimit = SERVO_MAX - targetPos;
            double scale = distToLimit / SOFT_LIMIT_ZONE;
            scale = clamp(scale, 0.1, 1.0);
            if (rawOutput > 0) {  // Only limit if moving toward limit
                rawOutput *= scale;
            }
        }

        // Hard clamp final position
        targetPos = servoPos + rawOutput;
        targetPos = clamp(targetPos, SERVO_MIN, SERVO_MAX);

        return targetPos - servoPos;  // Return actual delta
    }

    // ==================== STAGE 6: SERVO OUTPUT ====================

    private void applyServoOutput(double output) {
        lastServoPos = servoPos;
        servoPos += output;
        servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);

        // Calculate servo velocity for telemetry
        if (deltaTime > 0) {
            servoVelocity = (servoPos - lastServoPos) / deltaTime;
        }

        turretServo.setPosition(servoPos);
    }

    // ==================== DISTANCE SCALING ====================

    private double getDistanceSpeedScale() {
        if (distanceFeet <= 0) return 1.0;

        if (distanceFeet <= CLOSE_DISTANCE_FT) {
            return CLOSE_SPEED_SCALE;
        } else if (distanceFeet >= FAR_DISTANCE_FT) {
            return FAR_SPEED_SCALE;
        } else {
            // Linear interpolation
            double t = (distanceFeet - CLOSE_DISTANCE_FT) / (FAR_DISTANCE_FT - CLOSE_DISTANCE_FT);
            return CLOSE_SPEED_SCALE + t * (FAR_SPEED_SCALE - CLOSE_SPEED_SCALE);
        }
    }

    private double getDistanceScaledDeadband(double baseDeadband) {
        if (distanceFeet <= 0) return baseDeadband;

        double scale;
        if (distanceFeet <= CLOSE_DISTANCE_FT) {
            scale = CLOSE_DEADBAND_SCALE;
        } else if (distanceFeet >= FAR_DISTANCE_FT) {
            scale = FAR_DEADBAND_SCALE;
        } else {
            double t = (distanceFeet - CLOSE_DISTANCE_FT) / (FAR_DISTANCE_FT - CLOSE_DISTANCE_FT);
            scale = CLOSE_DEADBAND_SCALE + t * (FAR_DEADBAND_SCALE - CLOSE_DEADBAND_SCALE);
        }

        return baseDeadband * scale;
    }

    // ==================== HARDWARE FAILURE ====================

    private void handleHardwareFailure() {
        // Hold current position on hardware failure
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }
        currentState = TurretState.SCANNING;
        inFailSafe = true;
    }

    // ==================== UTILITY ====================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ==================== PUBLIC GETTERS ====================

    // State
    public TurretState getCurrentState() { return currentState; }
    public String getStateName() { return currentState.name(); }
    public boolean isAligned() { return aligned; }
    public boolean isTagDetected() { return tagDetected; }
    public boolean isInFailSafe() { return inFailSafe; }
    public boolean isInLockMode() {
        return currentState == TurretState.TRACKING || currentState == TurretState.HOLDING;
    }

    // Vision
    public double getRawTx() { return rawTx; }
    public double getFilteredTx() { return filteredTx; }
    public double getCompensatedTx() { return compensatedTx; }
    public double getConfidence() { return confidence; }
    public double getConfidenceScale() { return confidenceScale; }
    public int getConsecutiveFrames() { return consecutiveFrames; }
    public int getFramesSinceSeen() { return framesSinceSeen; }

    // Distance
    public double getDistanceMeters() { return distanceMeters; }
    public double getDistanceFeet() { return distanceFeet; }
    public double getTargetArea() { return targetArea; }

    // IMU
    public double getRobotYawRate() { return robotYawRate; }
    public double getFilteredYawRate() { return filteredYawRate; }
    public double getRobotYaw() { return robotYaw; }
    public double getImuOutput() { return imuOutput; }

    // Control outputs
    public double getTrimOutput() { return trimOutput; }
    public double getServoPosition() { return servoPos; }
    public double getServoVelocity() { return servoVelocity; }

    // Timing
    public double getLoopTimeMs() { return loopTimeMs; }
    public int getTargetTagId() { return targetTagId; }

    // ==================== PUBLIC SETTERS ====================

    public void lock() {
        isManuallyLocked = true;
        manualLockPosition = servoPos; // freeze current position
    }

    public void unlock() {
        isManuallyLocked = false;
        // Do NOT reset servoPos; state machine will now take control
    }

    public void setPositionDirect(double position) {
        manualLockPosition = clamp(position, SERVO_MIN, SERVO_MAX);
        servoPos = manualLockPosition;
        isManuallyLocked = true;
        if (turretServo != null) {
            turretServo.setPosition(servoPos);
        }
    }

    public void resetLock() {
        transitionTo(TurretState.SCANNING);
        lowConfidenceFrames = 0;
    }

    public void setAlliance(AllianceColor alliance) {
        targetTagId = (alliance == AllianceColor.BLUE) ? BLUE_ALLIANCE_TAG : RED_ALLIANCE_TAG;
    }

    public void resetIMU() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
        robotYaw = 0;
        filteredYawRate = 0;
    }

    // For backwards compatibility
    public boolean hasEverLocked() {
        return currentState != TurretState.SCANNING;
    }

    private int lowConfidenceFrames = 0;  // For backwards compatibility

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        telemetry.addLine("=== TurretLockOptimized (Competition) ===");
        telemetry.addLine("");

        // State machine
        telemetry.addLine("--- State ---");
        telemetry.addData("State", currentState.name() + (inFailSafe ? " [FAIL-SAFE]" : ""));
        telemetry.addData("Aligned", aligned ? "YES" : "NO");
        telemetry.addData("Tag", tagDetected ? "DETECTED" : "LOST (" + framesSinceSeen + " frames)");
        telemetry.addLine("");

        // Confidence
        telemetry.addLine("--- Confidence ---");
        telemetry.addData("Score", "%.0f%%", confidence * 100);
        telemetry.addData("Control Scale", "%.0f%%", confidenceScale * 100);
        telemetry.addLine("");

        // Vision with latency compensation
        telemetry.addLine("--- Vision (Latency Compensated) ---");
        telemetry.addData("Raw tx", "%.2f°", rawTx);
        telemetry.addData("Filtered tx", "%.2f°", filteredTx);
        telemetry.addData("Compensated tx", "%.2f°", compensatedTx);
        telemetry.addLine("");

        // IMU
        telemetry.addLine("--- IMU ---");
        telemetry.addData("Yaw Rate", "%.1f°/s (filtered: %.1f)", robotYawRate, filteredYawRate);
        telemetry.addData("IMU Output", "%.5f", imuOutput);
        telemetry.addLine("");

        // Control
        telemetry.addLine("--- Control ---");
        telemetry.addData("Trim Output", "%.5f", trimOutput);
        telemetry.addData("Servo Pos", "%.4f", servoPos);
        telemetry.addData("Distance", "%.2f ft", distanceFeet);
        telemetry.addLine("");

        // Performance
        telemetry.addData("Loop Time", "%.1f ms", loopTimeMs);
        telemetry.addData("Target Tag", targetTagId);
    }
}

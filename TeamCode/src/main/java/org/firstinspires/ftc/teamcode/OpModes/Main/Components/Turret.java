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

public class Turret {
    private Limelight3A limelight;
    private Servo turretServo;
    private Telemetry telemetry;

    // PID constants - tuned to prevent oscillation
    private double kP = 0.0004;  // Reduced to prevent overshoot
    private double kI = 0.0;
    private double kD = 0.0004; // Increased for better damping and oscillation prevention

    private double integral = 0;
    private double lastError = 0;
    private double secondLastError = 0; // Track error history for oscillation detection
    private int oscillationCount = 0; // Count oscillations (PID active = oscillation)
    
    // Data tracking for range adjustment
    private int pidActiveCycles = 0; // Count of PID active cycles
    private double minErrorDuringPID = Double.MAX_VALUE; // Minimum error seen during PID
    private double maxErrorDuringPID = Double.MIN_VALUE; // Maximum error seen during PID
    private int samplesNearRange = 0; // Count samples that are close to ideal range but not in it
    private static final double NEAR_RANGE_THRESHOLD = 1.5; // Consider "near" if within 1.5x ideal range

    // Servo config
    private static final double SERVO_MIN = 0.375;
    private static final double SERVO_MAX = 0.8;
    private static final double TARGET_TX = 0.0;  // ideal target is zero (centered)
    private static final double IDEAL_RANGE = 3.0; // very tight tolerance for precise accuracy
    private boolean CHECK = false;
    private double servoPos = 0.5;
    private boolean aligned = false;
    private boolean appliedLeadOffset = false;
    private boolean lastFrameLocked = false;
    // Direction memory
    private int lastDirection = 1; // 1 = right, -1 = left

    // Scanning speed and behavior - FAST when not looking at tag
    private double scanSpeed = 0.003; // Increased for faster scanning when tag not detected
    private double searchRange = 0.1; // how far to scan around last seen position
    private int framesSinceSeen = 0;   // expands search zone over time if lost
    private double lockedServoPos = 0.5;

    // Max incremental adjustment - slightly increased for faster convergence
    private static final double MAX_OUTPUT = 0.0007;

    // Servo direction multiplier
    private double servoDirection = 1.0;
    
    // Lock flag to prevent update() from running when turret is locked
    private boolean isLocked = false;
    private double lockedPosition = 0.5;

    // Target AprilTag ID based on alliance color (20 for BLUE, 24 for RED)
    private int targetTagId = 24; // Default to RED for backward compatibility

    // Limelight constants for distance calculation
    private static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032;
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;   // Limelight 3A vertical FOV
    private static final int IMAGE_WIDTH_PIXELS = 1280;
    private static final int IMAGE_HEIGHT_PIXELS = 720;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        initialize(hardwareMap, telemetry, null);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, HardwareConfig.LIMELIGHT);
        turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);

        // Set target tag ID based on alliance color (default to RED/24 if null)
        if (allianceColor == AllianceColor.BLUE) {
            targetTagId = 20;
        } else {
            targetTagId = 24; // RED or default
        }

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }

        turretServo.setPosition(servoPos);
        telemetry.addLine("✅ Turret PID Align Initialized");
    }

    public boolean update() {
        // If turret is locked, skip all update logic and just maintain position
        if (isLocked) {
            turretServo.setPosition(lockedPosition);
            return true;
        }
        
        if (limelight == null || !limelight.isConnected()) {
            telemetry.addLine("❌ Limelight not connected");
            telemetry.update();
            return false; // Signal that OpMode should exit
        }

        LLResult result = limelight.getLatestResult();
        double tx = 0;
        boolean tagDetected = false;

        // Detect target tag based on alliance color (20 for BLUE, 24 for RED)
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == targetTagId) {
                    tx = fr.getTargetXDegrees();
                    tagDetected = true;
                    break;
                }
            }
        }

        if (tagDetected) {
            framesSinceSeen = 0;
            double error = tx - TARGET_TX;

            // Dynamically expand range when oscillating to help it lock sooner
            // Expand range gradually based on oscillation count
            double dynamicRange = IDEAL_RANGE;
            if (pidActiveCycles > 0) {
                // Expand range by 20% for each oscillation cycle, up to 3x the original range
                double expansionFactor = 1.0 + (pidActiveCycles * 0.2);
                expansionFactor = Math.min(expansionFactor, 3.0); // Cap at 3x
                dynamicRange = IDEAL_RANGE * expansionFactor;
            }
            
            // Always check if we're in the dynamic range (continuous monitoring)
            boolean inIdealRange = Math.abs(error) <= dynamicRange;

            // ---------------- LOCK BEHAVIOR ----------------
            if (inIdealRange) {
                aligned = true;
                // Hold current position while locked
                turretServo.setPosition(servoPos);

                // Reset PID terms when locked
                integral = 0;
                lastError = 0;
                secondLastError = 0;
                oscillationCount = 0;
                pidActiveCycles = 0;
                minErrorDuringPID = Double.MAX_VALUE;
                maxErrorDuringPID = Double.MIN_VALUE;
                samplesNearRange = 0;
                lastFrameLocked = true;
                CHECK = true;

                telemetry.addLine("🎯 Aligned! Holding position");
                telemetry.addData("tx", tx);
                telemetry.addData("Error", error);
                telemetry.addData("Dynamic Range Used", "%.2f", dynamicRange);
                telemetry.addData("Servo Pos", servoPos);
                telemetry.addData("In Range", "YES");

            } else {
                // ---------------- PID ACTIVE ----------------
                aligned = false;
                appliedLeadOffset = false;
                CHECK = false; // Not in range, so clear CHECK flag

                // Track data during PID to determine if range needs expansion
                double errorMagnitude = Math.abs(error);
                if (errorMagnitude < minErrorDuringPID) {
                    minErrorDuringPID = errorMagnitude;
                }
                if (errorMagnitude > maxErrorDuringPID) {
                    maxErrorDuringPID = errorMagnitude;
                }
                
                // Count samples that are close to ideal range but not in it
                if (errorMagnitude > IDEAL_RANGE && errorMagnitude <= IDEAL_RANGE * NEAR_RANGE_THRESHOLD) {
                    samplesNearRange++;
                }

                // Detect error direction changes for additional damping
                boolean errorDirectionChanged = (error > 0 && lastError < 0) || (error < 0 && lastError > 0);
                
                // Calculate oscillation damping based on current cycle count
                // Will be updated after we determine if we're making a correction
                double oscillationDamping = 1.0;

                // Reset integral when direction changes to prevent windup
                if (errorDirectionChanged) {
                    integral = 0;
                }

                // Calculate PID output
                integral += error;
                // Limit integral to prevent windup
                double maxIntegral = 50.0;
                integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
                
                double derivative = error - lastError;
                double rawOutput = (kP * error) + (kI * integral) + (kD * derivative);
                
                // Scale speed based on error magnitude (MUCH slower when closer)
                // Use cubic/quartic scaling for extremely aggressive slowdown near target
                errorMagnitude = Math.abs(error);

                // Base speed reduction when tag is detected
                double baseSpeedReduction = 0.2; // Reduce base speed to 25% when tag detected

                // Non-linear scaling: use quartic (4th power) error for extremely dramatic slowdown when close
                // When error is large, speed is higher; when error is small, speed is MUCH lower
                double maxErrorForScaling = 20.0; // Scale based on this max error
                double normalizedError = Math.min(1.0, errorMagnitude / maxErrorForScaling);

                // Quartic scaling (4th power): 
                // when error is 0.5, speed is 0.0625 (6.25%)
                // when error is 0.2, speed is 0.0016 (0.16%)
                // when error is 0.1, speed is 0.0001 (0.01%)
                double speedScale = baseSpeedReduction * Math.pow(normalizedError, 4);

                // Ensure minimum speed when very close (but still allow movement)
                double minSpeedScale = 0.02; // Minimum 2% speed even when very close
                speedScale = Math.max(minSpeedScale, speedScale);

                // Apply speed scaling and clamp output
                double scaledOutput = rawOutput * speedScale;
                
                double output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, scaledOutput));

                // Determine direction: negative error = move left (decrease servo), positive error = move right (increase servo)
                // Error = tx - 0 = tx, so:
                //   If tx < 0 (target left), error < 0 → need to decrease servo (move left)
                //   If tx > 0 (target right), error > 0 → need to increase servo (move right)
                double step = servoDirection * output;

                // Prevent overshooting: when close to target, limit step size to avoid skipping over ideal range
                // More aggressive limiting when very close OR when oscillating
                if (Math.abs(error) <= IDEAL_RANGE * 2 || pidActiveCycles >= 2) {
                    // When close OR oscillating, limit movement more aggressively
                    double proximityFactor = Math.abs(error) / (IDEAL_RANGE * 2); // 0 to 1
                    double oscillationFactor = pidActiveCycles >= 2 ? 0.3 : 1.0; // Extra reduction when oscillating
                    double maxStep = Math.abs(error) * 0.3 * proximityFactor * oscillationFactor;
                    step = Math.signum(step) * Math.min(Math.abs(step), maxStep);
                }

                // Count oscillations more accurately:
                // 1. Count when there's any movement (lower threshold)
                // 2. Count when error direction changes (oscillating back and forth)
                boolean shouldCountOscillation = false;
                
                if (Math.abs(step) > 0.0001) {
                    // Any movement counts
                    shouldCountOscillation = true;
                } else if (errorDirectionChanged && pidActiveCycles > 0) {
                    // Error direction change also counts (oscillating behavior)
                    shouldCountOscillation = true;
                }
                
                if (shouldCountOscillation) {
                    // COUNT THIS AS AN OSCILLATION CYCLE
                    pidActiveCycles++;
                    oscillationCount = pidActiveCycles;
                    
                    // Cap oscillation count to prevent overflow
                    if (oscillationCount > 20) {
                        oscillationCount = 20;
                        pidActiveCycles = 20;
                    }
                }
                
                // Apply oscillation damping - HALVE SPEED each cycle
                // Cycle 1: 100%, Cycle 2: 50%, Cycle 3: 25%, Cycle 4: 12.5%, etc.
                if (pidActiveCycles > 0) {
                    oscillationDamping = Math.pow(0.5, pidActiveCycles - 1);
                    
                    // Set minimum damping to prevent it from going too slow
                    double minDamping = 0.05; // Minimum 5% speed
                    oscillationDamping = Math.max(minDamping, oscillationDamping);
                    
                    // Apply damping to step before moving
                    step *= oscillationDamping;
                }
                
                servoPos += step;
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(servoPos);

                // CONSTANTLY check if we're now in range after this correction step
                // This catches the moment it enters the ideal range during oscillation
                // Use dynamic range that expands with oscillation
                double newError = tx - TARGET_TX; // Re-check error (tx may have changed)
                double currentDynamicRange = IDEAL_RANGE;
                if (pidActiveCycles > 0) {
                    double expansionFactor = 1.0 + (pidActiveCycles * 0.2);
                    expansionFactor = Math.min(expansionFactor, 3.0);
                    currentDynamicRange = IDEAL_RANGE * expansionFactor;
                }
                boolean nowInRange = Math.abs(newError) <= currentDynamicRange;
                
                if (nowInRange) {
                    // We've entered the ideal range! Lock immediately
                    aligned = true;
                    integral = 0;
                    lastError = 0;
                    secondLastError = 0;
                    oscillationCount = 0;
                    pidActiveCycles = 0;
                    minErrorDuringPID = Double.MAX_VALUE;
                    maxErrorDuringPID = Double.MIN_VALUE;
                    samplesNearRange = 0;
                    lastFrameLocked = true;
                    CHECK = true;
                    
                    telemetry.addLine("🎯 Entered Range! Locking position");
                    telemetry.addData("tx", tx);
                    telemetry.addData("Error", newError);
                    telemetry.addData("Dynamic Range Used", "%.2f", currentDynamicRange);
                    telemetry.addData("Oscillation Cycles", pidActiveCycles);
                    telemetry.addData("Servo Pos", servoPos);
                    telemetry.addData("In Range", "YES");
                } else {
                    // Still correcting, continue PID
                    // Update direction tracking
                    lastDirection = step > 0 ? 1 : -1;
                    
                    // Update error history for oscillation detection
                    secondLastError = lastError;
                    lastError = error;
                    lastFrameLocked = true;

                    telemetry.addLine("⚡ PID Active: Correcting Position");
                    telemetry.addData("tx", tx);
                    telemetry.addData("Error", error);
                    telemetry.addData("Speed Scale", "%.3f", speedScale);
                    telemetry.addData("PID Cycles (Oscillation)", pidActiveCycles);
                    telemetry.addData("Damping", "%.1f%%", oscillationDamping * 100);
                    telemetry.addData("Dynamic Range", "%.2f (Base: %.2f)", dynamicRange, IDEAL_RANGE);
                    telemetry.addData("Servo Pos", servoPos);
                    telemetry.addData("In Range", "NO");
                    
                    // Display range analysis data
                    telemetry.addLine("");
                    telemetry.addLine("=== RANGE ANALYSIS ===");
                    telemetry.addData("Min Error During PID", "%.2f", minErrorDuringPID);
                    telemetry.addData("Max Error During PID", "%.2f", maxErrorDuringPID);
                    telemetry.addData("Current Ideal Range", "%.2f", IDEAL_RANGE);
                    telemetry.addData("Samples Near Range", samplesNearRange);
                    
                    // Suggest range expansion if oscillating around a value close to current range
                    if (pidActiveCycles > 10 && samplesNearRange > pidActiveCycles * 0.3) {
                        // If 30%+ of samples are near range, suggest expanding
                        double suggestedRange = Math.max(IDEAL_RANGE, minErrorDuringPID * 1.2);
                        telemetry.addData("⚠️ Suggested Range", "%.2f", suggestedRange);
                        telemetry.addLine("(Many samples near current range)");
                    }
                }
            }


            lockedServoPos = servoPos;

        } else {
            // ---------------- TAG LOST ----------------
            // Reset oscillation count when tag is lost (start fresh when scanning)
            pidActiveCycles = 0;
            oscillationCount = 0;
            minErrorDuringPID = Double.MAX_VALUE;
            maxErrorDuringPID = Double.MIN_VALUE;
            samplesNearRange = 0;
            
            framesSinceSeen++;
            integral = 0;
            aligned = false;
            appliedLeadOffset = false;

            // Reverse search direction if just lost
            if (lastFrameLocked) {
                lastDirection *= -1;
                lockedServoPos = servoPos; // start scanning from lost position
            }

            // Smart scanning within dynamic range
            double dynamicRange = Math.min(0.3, searchRange + framesSinceSeen * 0.005);
            double nextPos = servoPos + lastDirection * scanSpeed;

            if (nextPos > Math.min(SERVO_MAX, lockedServoPos + dynamicRange)) {
                lastDirection = -1;
                nextPos = Math.min(SERVO_MAX, lockedServoPos + dynamicRange);
            } else if (nextPos < Math.max(SERVO_MIN, lockedServoPos - dynamicRange)) {
                lastDirection = 1;
                nextPos = Math.max(SERVO_MIN, lockedServoPos - dynamicRange);
            }

            servoPos = nextPos;
            turretServo.setPosition(servoPos);

            telemetry.addLine("🔍 No Tag Detected — Smart Scanning");
            telemetry.addData("Servo Pos", servoPos);
            telemetry.addData("Search Range", dynamicRange);
            telemetry.addData("Scan Direction", lastDirection == 1 ? "Right" : "Left");
            lastFrameLocked = false;
        }
        return true; // Continue OpMode
    }
    
    /**
     * Set turret to a fixed position directly (bypasses scanning/alignment)
     * Locks the turret at this position and prevents update() from changing it
     * @param position Servo position (0.0 to 1.0)
     */
    public void setPositionDirect(double position) {
        // Clamp position to valid range
        position = Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
        servoPos = position;
        lockedPosition = position;
        isLocked = true; // Lock turret to prevent update() from changing position
        turretServo.setPosition(servoPos);
    }
    
    /**
     * Unlock turret to allow update() method to control it again
     */
    public void unlock() {
        isLocked = false;
    }

    /**
     * Calculate distance to AprilTag target using TA (Target Area) method
     * @return Distance in feet, or -1.0 if no valid target detected
     */
    public double getDistance() {
        if (limelight == null || !limelight.isConnected()) {
            return -1.0;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double taPercent = result.getTa();
            if (taPercent > 0.0) {
                double pixelArea = (taPercent / 100.0) * (IMAGE_WIDTH_PIXELS * IMAGE_HEIGHT_PIXELS);
                double tagPixelHeight = Math.sqrt(pixelArea);
                double focalPx = (IMAGE_HEIGHT_PIXELS / 2.0)
                        / Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV_DEGREES / 2.0));

                double distanceMeters = (APRILTAG_REAL_HEIGHT_METERS * focalPx) / tagPixelHeight;
                return distanceMeters * 3.28084; // Convert to feet
            }
        }
        return -1.0;
    }

    /**
     * Detects obelisk AprilTag IDs (21, 22, 23) on the obelisk
     * @return The detected AprilTag ID (21, 22, or 23), or -1 if none detected
     */
    public int detectObeliskAprilTag() {
        if (limelight == null || !limelight.isConnected()) {
            return -1;
        }

        try {
            // Get latest detection results from Limelight
            LLResult result = limelight.getLatestResult();

            // Verify result is valid and not null
            if (result != null && result.isValid()) {
                // Get all detected fiducials (AprilTags)
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    // Check each detected fiducial for obelisk tags
                    for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                        int tagId = fiducial.getFiducialId();
                        
                        // Check if this is an obelisk tag (21, 22, or 23)
                        if (tagId == 21 || tagId == 22 || tagId == 23) {
                            return tagId;
                        }
                    }
                }
            }
        } catch (Exception e) {
            // Handle any errors gracefully
            return -1;
        }
        
        return -1;
    }
}


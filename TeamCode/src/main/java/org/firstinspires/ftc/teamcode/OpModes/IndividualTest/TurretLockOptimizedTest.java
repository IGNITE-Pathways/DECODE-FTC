package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.TurretLockOptimized.TurretState;

/**
 * Test OpMode for TurretLockOptimized - Competition-Level Turret Testing
 *
 * This tests all advanced features of the competition turret:
 * - Latency compensation (predicts current tag position)
 * - Multi-stage state machine (SCANNING -> ACQUIRING -> TRACKING -> HOLDING)
 * - Confidence-based control (low confidence = slower response)
 * - Soft rotation limits (smooth slowdown at edges)
 * - Distance-based speed scaling
 * - Fail-safe behavior (holds position when vision drops)
 * - IMU counter-rotation + vision trim
 *
 * CONTROLS:
 *   A: Start/Reset test
 *   B: Force offset (test recovery)
 *   X: Toggle RED/BLUE alliance
 *   Y: Lock/Unlock turret manually
 *   Left Bumper: Reset IMU yaw
 *   Right Bumper: Force fail-safe test (block camera briefly)
 *   D-Pad: Manual servo adjustment
 *
 * TEST PROCEDURE:
 * 1. Point at AprilTag and press A to start
 * 2. Watch state transitions: SCANNING -> ACQUIRING -> TRACKING -> HOLDING
 * 3. Spin the robot - turret should counter-rotate to maintain lock
 * 4. Block the camera briefly - should enter FAIL-SAFE and hold position
 * 5. Press B to force an offset and test recovery speed
 */
@TeleOp(name = "Test: TurretLockOptimized", group = "Test")
public class TurretLockOptimizedTest extends LinearOpMode {

    private TurretLockOptimized turret;
    private AllianceColor alliance = AllianceColor.BLUE;

    // Test metrics
    private ElapsedTime testTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    // Performance tracking
    private double timeToAcquire = -1;
    private double timeToTrack = -1;
    private double timeToHold = -1;
    private double maxYawRateWhileLocked = 0;
    private double maxConfidence = 0;
    private int failSafeCount = 0;
    private double totalFailSafeTime = 0;
    private double failSafeStartTime = 0;
    private boolean wasInFailSafe = false;

    // State tracking
    private TurretState lastState = TurretState.SCANNING;
    private boolean testRunning = false;

    // Button edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLB = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {
        // Initialize
        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, alliance);

        telemetry.addLine("=== TurretLockOptimized Competition Test ===");
        telemetry.addLine("");
        telemetry.addLine("FEATURES BEING TESTED:");
        telemetry.addLine("  - Latency compensation");
        telemetry.addLine("  - 4-stage state machine");
        telemetry.addLine("  - Confidence-based control");
        telemetry.addLine("  - Soft rotation limits");
        telemetry.addLine("  - Distance-based scaling");
        telemetry.addLine("  - Fail-safe hold behavior");
        telemetry.addLine("  - IMU counter-rotation");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: Start/Reset | B: Force offset");
        telemetry.addLine("  X: Toggle alliance | Y: Lock/Unlock");
        telemetry.addLine("  LB: Reset IMU | D-Pad: Manual");
        telemetry.addLine("");
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        testTimer.reset();
        stateTimer.reset();

        while (opModeIsActive()) {
            // Handle inputs
            handleInputs();

            // Update turret
            turret.update();

            // Track metrics
            if (testRunning) {
                updateMetrics();
            }

            // Display telemetry
            displayTelemetry();

            idle();
        }
    }

    private void handleInputs() {
        // A: Start/Reset test
        if (gamepad1.a && !prevA) {
            startTest();
        }
        prevA = gamepad1.a;

        // B: Force offset
        if (gamepad1.b && !prevB) {
            forceOffset();
        }
        prevB = gamepad1.b;

        // X: Toggle alliance
        if (gamepad1.x && !prevX) {
            toggleAlliance();
        }
        prevX = gamepad1.x;

        // Y: Lock/Unlock
        if (gamepad1.y && !prevY) {
            toggleLock();
        }
        prevY = gamepad1.y;

        // Left Bumper: Reset IMU
        if (gamepad1.left_bumper && !prevLB) {
            turret.resetIMU();
        }
        prevLB = gamepad1.left_bumper;

        // D-Pad: Manual adjustment
        if (gamepad1.dpad_left) {
            turret.setPositionDirect(turret.getServoPosition() - 0.01);
            sleep(50);
            turret.unlock();
        }
        if (gamepad1.dpad_right) {
            turret.setPositionDirect(turret.getServoPosition() + 0.01);
            sleep(50);
            turret.unlock();
        }
    }

    private void startTest() {
        testRunning = true;
        testTimer.reset();
        stateTimer.reset();

        // Reset metrics
        timeToAcquire = -1;
        timeToTrack = -1;
        timeToHold = -1;
        maxYawRateWhileLocked = 0;
        maxConfidence = 0;
        failSafeCount = 0;
        totalFailSafeTime = 0;
        wasInFailSafe = false;
        lastState = TurretState.SCANNING;

        turret.unlock();
        turret.resetLock();
        turret.resetIMU();
    }

    private void forceOffset() {
        turret.setPositionDirect(0.4);
        sleep(100);
        turret.unlock();
        stateTimer.reset();

        // Reset timing metrics for recovery test
        timeToAcquire = -1;
        timeToTrack = -1;
        timeToHold = -1;
    }

    private void toggleAlliance() {
        if (alliance == AllianceColor.RED) {
            alliance = AllianceColor.BLUE;
        } else {
            alliance = AllianceColor.RED;
        }
        turret.setAlliance(alliance);
        startTest();
    }

    private void toggleLock() {
        if (turret.isInLockMode()) {
            turret.unlock(); // if locked, unlock
        } else {
            turret.lock();   // if unlocked, lock
        }
    }

    private void updateMetrics() {
        TurretState currentState = turret.getCurrentState();
        double currentTime = testTimer.seconds();

        // Track state transition times
        if (currentState != lastState) {
            if (currentState == TurretState.ACQUIRING && timeToAcquire < 0) {
                timeToAcquire = currentTime;
            } else if (currentState == TurretState.TRACKING && timeToTrack < 0) {
                timeToTrack = currentTime;
            } else if (currentState == TurretState.HOLDING && timeToHold < 0) {
                timeToHold = currentTime;
            }
            lastState = currentState;
        }

        // Track max confidence
        if (turret.getConfidence() > maxConfidence) {
            maxConfidence = turret.getConfidence();
        }

        // Track max rotation speed while maintaining lock
        if (turret.isInLockMode() && turret.isAligned()) {
            double currentYawRate = Math.abs(turret.getRobotYawRate());
            if (currentYawRate > maxYawRateWhileLocked) {
                maxYawRateWhileLocked = currentYawRate;
            }
        }

        // Track fail-safe occurrences
        if (turret.isInFailSafe() && !wasInFailSafe) {
            failSafeCount++;
            failSafeStartTime = currentTime;
        }
        if (!turret.isInFailSafe() && wasInFailSafe) {
            totalFailSafeTime += (currentTime - failSafeStartTime);
        }
        wasInFailSafe = turret.isInFailSafe();
    }

    private void displayTelemetry() {
        telemetry.addLine("======== TURRET LOCK OPTIMIZED TEST ========");
        telemetry.addLine("        (Competition-Level Features)");
        telemetry.addLine("");

        // Current State (prominent display)
        TurretState state = turret.getCurrentState();
        String stateDisplay = getStateDisplay(state);
        telemetry.addLine(">>> STATE: " + stateDisplay + " <<<");
        if (turret.isInFailSafe()) {
            telemetry.addLine("    !! FAIL-SAFE ACTIVE !!");
        }
        telemetry.addLine("");

        // Lock & Alignment
        telemetry.addLine("--- Lock Status ---");
        telemetry.addData("Aligned", turret.isAligned() ? "YES" : "NO");
        telemetry.addData("Tag Detected", turret.isTagDetected() ? "YES" : "NO");
        telemetry.addLine("");

        // Confidence (key metric)
        telemetry.addLine("--- Confidence ---");
        telemetry.addData("Score", "%.0f%%", turret.getConfidence() * 100);
        telemetry.addData("Control Authority", "%.0f%%", turret.getConfidenceScale() * 100);
        telemetry.addLine("");

        // Latency Compensation
        telemetry.addLine("--- Latency Compensation ---");
        telemetry.addData("Raw tx", "%.2f°", turret.getRawTx());
        telemetry.addData("Compensated tx", "%.2f°", turret.getCompensatedTx());
        double compensation = turret.getCompensatedTx() - turret.getFilteredTx();
        telemetry.addData("Correction Applied", "%.2f°", compensation);
        telemetry.addLine("");

        // IMU Counter-Rotation
        telemetry.addLine("--- IMU Counter-Rotation ---");
        telemetry.addData("Yaw Rate", "%.1f°/s", turret.getRobotYawRate());
        telemetry.addData("Filtered Rate", "%.1f°/s", turret.getFilteredYawRate());
        telemetry.addData("IMU Output", "%.5f", turret.getImuOutput());
        telemetry.addLine("");

        // Vision Trim
        telemetry.addLine("--- Vision Trim ---");
        telemetry.addData("Trim Output", "%.5f", turret.getTrimOutput());
        telemetry.addLine("");

        // Test Metrics
        if (testRunning) {
            telemetry.addLine("--- Test Metrics ---");
            telemetry.addData("Test Time", "%.2f sec", testTimer.seconds());

            // State transition times
            if (timeToAcquire > 0) {
                telemetry.addData("Time to ACQUIRE", "%.3f sec", timeToAcquire);
            }
            if (timeToTrack > 0) {
                telemetry.addData("Time to TRACK", "%.3f sec", timeToTrack);
            }
            if (timeToHold > 0) {
                telemetry.addData("Time to HOLD", "%.3f sec", timeToHold);
                String grade = gradeHoldTime(timeToHold);
                telemetry.addData("Performance", grade);
            }
            telemetry.addLine("");

            telemetry.addLine("--- Stability ---");
            telemetry.addData("Max Confidence", "%.0f%%", maxConfidence * 100);
            telemetry.addData("Max Yaw Rate (Locked)", "%.1f°/s", maxYawRateWhileLocked);
            telemetry.addData("Fail-Safe Count", failSafeCount);
            if (totalFailSafeTime > 0) {
                telemetry.addData("Total Fail-Safe Time", "%.2f sec", totalFailSafeTime);
            }
            telemetry.addLine("");
        }

        // Servo & Distance
        telemetry.addLine("--- Output ---");
        telemetry.addData("Servo Position", "%.4f", turret.getServoPosition());
        telemetry.addData("Distance", "%.2f ft", turret.getDistanceFeet());
        telemetry.addData("Loop Time", "%.1f ms", turret.getLoopTimeMs());
        telemetry.addData("Alliance", alliance + " (Tag " + turret.getTargetTagId() + ")");
        telemetry.addLine("");

        // Controls reminder
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A=Test | B=Offset | X=Alliance | Y=Lock");
        telemetry.addLine("LB=ResetIMU | D-Pad=Manual");
        telemetry.addLine("");
        telemetry.addLine("TIP: Spin robot while TRACKING to test");
        telemetry.addLine("IMU counter-rotation!");

        telemetry.update();
    }

    private String getStateDisplay(TurretState state) {
        switch (state) {
            case SCANNING:
                return "SCANNING (looking for tag...)";
            case ACQUIRING:
                return "ACQUIRING (moving to tag)";
            case TRACKING:
                return "TRACKING (IMU + vision active)";
            case HOLDING:
                return "HOLDING (locked, minimal power)";
            default:
                return state.name();
        }
    }

    private String gradeHoldTime(double time) {
        if (time < 0.4) return "EXCELLENT (<0.4s)";
        if (time < 0.7) return "GREAT (<0.7s)";
        if (time < 1.0) return "GOOD (<1.0s)";
        if (time < 1.5) return "ACCEPTABLE (<1.5s)";
        return "SLOW - needs tuning";
    }
}

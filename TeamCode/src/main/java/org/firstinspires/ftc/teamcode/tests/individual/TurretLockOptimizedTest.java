package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;

/**
 * Test OpMode for TurretLockOptimized - SIMPLIFIED VERSION
 *
 * CONTROLS:
 *   A: Start/Reset test
 *   B: Force offset (test recovery)
 *   X: Toggle RED/BLUE alliance
 *   Y: Lock/Unlock turret manually
 *   D-Pad Left/Right: Manual servo adjustment
 *
 * STATES:
 *   SCANNING  - Looking for AprilTag (sweeping back and forth)
 *   TRACKING  - Tag found, moving toward center
 *   LOCKED    - Tag is centered, holding position
 *   MANUAL    - User locked the turret manually
 */
@TeleOp(name = "Test: TurretLockOptimized", group = "Test")
public class TurretLockOptimizedTest extends LinearOpMode {

    private TurretLockOptimized turret;
    private AllianceColor alliance = AllianceColor.BLUE;

    // Test metrics
    private ElapsedTime testTimer = new ElapsedTime();
    private double timeToLock = -1;
    private boolean wasLocked = false;

    // Button edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {
        // Initialize
        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, alliance);

        telemetry.addLine("=== TurretLockOptimized Test ===");
        telemetry.addLine("");
        telemetry.addLine("SIMPLE TURRET - Only 4 tuning values!");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: Start/Reset test");
        telemetry.addLine("  B: Force offset (test recovery)");
        telemetry.addLine("  X: Toggle alliance");
        telemetry.addLine("  Y: Lock/Unlock turret");
        telemetry.addLine("  D-Pad: Manual adjustment");
        telemetry.addLine("");
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        testTimer.reset();

        while (opModeIsActive()) {
            // Handle inputs
            handleInputs();

            // Update turret
            turret.update();

            // Track time to lock
            if (turret.isLocked() && !wasLocked && timeToLock < 0) {
                timeToLock = testTimer.seconds();
            }
            wasLocked = turret.isLocked();

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
        testTimer.reset();
        timeToLock = -1;
        wasLocked = false;
        turret.resetLock();
    }

    private void forceOffset() {
        turret.setPositionDirect(0.4);
        sleep(100);
        turret.unlock();
        testTimer.reset();
        timeToLock = -1;
        wasLocked = false;
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
        if (turret.isLocked()) {
            turret.unlock();
        } else {
            turret.lock();
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("======== TURRET LOCK TEST ========");
        telemetry.addLine("");

        // Current State (big and clear)
        String state = turret.getStateName();
        telemetry.addLine(">>> STATE: " + state + " <<<");
        telemetry.addLine("");

        // Status
        telemetry.addLine("--- Status ---");
        telemetry.addData("Tag Visible", turret.isTagVisible() ? "YES" : "NO");
        telemetry.addData("Locked On", turret.isLocked() ? "YES" : "NO");
        telemetry.addLine("");

        // Vision Data
        telemetry.addLine("--- Vision ---");
        telemetry.addData("tx (current)", "%.1f°", turret.getTx());
        telemetry.addData("tx (predicted)", "%.1f°", turret.getPredictedTx());
        telemetry.addData("tx velocity", "%.1f°/s", turret.getTxVelocity());
        telemetry.addLine("");

        // Control Output
        telemetry.addLine("--- Control ---");
        telemetry.addData("Position term", "%.5f", turret.getPositionComponent());
        telemetry.addData("Velocity term", "%.5f", turret.getVelocityComponent());
        telemetry.addData("Total output", "%.5f", turret.getLastMovement());
        telemetry.addLine("");

        // Servo
        telemetry.addLine("--- Servo ---");
        telemetry.addData("Position", "%.3f", turret.getServoPosition());
        telemetry.addLine("");

        // Test Metrics
        telemetry.addLine("--- Test Metrics ---");
        telemetry.addData("Test Time", "%.2f sec", testTimer.seconds());
        if (timeToLock > 0) {
            telemetry.addData("Time to Lock", "%.3f sec", timeToLock);
            String grade = gradeLockTime(timeToLock);
            telemetry.addData("Performance", grade);
        } else if (turret.isTagVisible()) {
            telemetry.addLine("Waiting for lock...");
        } else {
            telemetry.addLine("Searching for tag...");
        }
        telemetry.addLine("");

        // Config
        telemetry.addLine("--- Config ---");
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Target Tag", turret.getTargetTagId());
        telemetry.addLine("");

        // DEBUG - What tags are actually being detected
        telemetry.addLine("--- DEBUG (IMPORTANT) ---");
        telemetry.addData("Limelight", turret.isLimelightConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData("Result Valid", turret.isResultValid() ? "YES" : "NO");
        telemetry.addData("Tags Found", turret.getDetectedTagCount());
        telemetry.addData("Tag IDs Seen", turret.getDetectedTagIds());
        telemetry.addData("Looking For", "Tag " + turret.getTargetTagId());
        telemetry.addLine("");

        // Controls
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A=Reset | B=Offset | X=Alliance | Y=Lock");
        telemetry.addLine("D-Pad Left/Right = Manual adjust");

        telemetry.update();
    }

    private String gradeLockTime(double time) {
        if (time < 0.3) return "EXCELLENT";
        if (time < 0.5) return "GREAT";
        if (time < 0.8) return "GOOD";
        if (time < 1.2) return "OK";
        return "SLOW - increase KP";
    }
}

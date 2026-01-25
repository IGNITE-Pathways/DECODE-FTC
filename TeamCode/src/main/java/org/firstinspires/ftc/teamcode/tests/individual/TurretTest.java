package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Turret Test with Drive
 *
 * Tests turret auto-tracking while allowing you to drive around.
 *
 * CONTROLS:
 *   Left Stick   = Drive forward/back and strafe
 *   Right Stick  = Rotate
 *   A            = Toggle turret auto-track ON/OFF
 *   B            = Switch alliance (Blue/Red tag)
 *   Y            = Reset turret to center
 *   LB           = Slow drive mode
 *   RB           = Fast drive mode
 *   D-Pad L/R    = Manual turret adjust (when auto-track OFF)
 */
@TeleOp(name = "Test: Turret + Drive", group = "Individual Test")
public class TurretTest extends OpMode {

    // Turret (manages both turret and drive)
    private TurretLockOptimized turret;
    private Servo turretServo;

    // State
    private boolean autoTrackEnabled = true;
    private AllianceColor alliance = AllianceColor.BLUE;
    private double manualServoPos = 0.5;  // Start at center

    // Speed mode: 0 = normal (65%), 1 = slow (35%), 2 = fast (95%)
    private int speedMode = 0;

    // Button edge detection
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    // Manual servo control
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.9;
    private static final double MANUAL_INCREMENT = 0.01;

    @Override
    public void init() {
        // Initialize turret (also initializes drive motors internally)
        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, alliance);

        // Initialize turret servo for manual control
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
        } catch (Exception e) {
            turretServo = null;
        }

        telemetry.addLine("=== TURRET + DRIVE TEST ===");
        telemetry.addLine("A = Toggle auto-track");
        telemetry.addLine("B = Switch alliance");
        telemetry.addLine("Y = Reset turret");
        telemetry.addLine("LB/RB = Speed modes");
        telemetry.addLine("Drive: " + (turret.isDriveInitialized() ? "OK" : "FAILED"));
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== DRIVING ==========
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Speed mode toggles
        if (gamepad1.left_bumper && !lastLB) {
            speedMode = (speedMode == 1) ? 0 : 1;
        }
        if (gamepad1.right_bumper && !lastRB) {
            speedMode = (speedMode == 2) ? 0 : 2;
        }
        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;

        // Apply speed multiplier
        double speedMult = (speedMode == 1) ? 0.35 : (speedMode == 2) ? 0.95 : 0.65;

        // Use turret's built-in drive control
        turret.drive(str * speedMult, fwd * speedMult, rot * speedMult);

        // ========== TURRET CONTROLS ==========

        // A = Toggle auto-track
        if (gamepad1.a && !lastA) {
            autoTrackEnabled = !autoTrackEnabled;
        }
        lastA = gamepad1.a;

        // B = Switch alliance
        if (gamepad1.b && !lastB) {
            alliance = (alliance == AllianceColor.BLUE) ? AllianceColor.RED : AllianceColor.BLUE;
            turret.setAlliance(alliance);
        }
        lastB = gamepad1.b;

        // Y = Reset turret to center
        if (gamepad1.y && !lastY) {
            turret.resetLock();
            manualServoPos = 0.5;  // Center position
        }
        lastY = gamepad1.y;

        // ========== TURRET UPDATE ==========
        if (autoTrackEnabled) {
            // Auto-tracking mode
            turret.update();
        } else {
            // Manual mode - D-Pad controls
            if (gamepad1.dpad_left) {
                manualServoPos -= MANUAL_INCREMENT;
            }
            if (gamepad1.dpad_right) {
                manualServoPos += MANUAL_INCREMENT;
            }
            // Right stick X for smooth control
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                manualServoPos += gamepad1.right_stick_x * 0.005;
            }

            manualServoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, manualServoPos));

            if (turretServo != null) {
                turretServo.setPosition(manualServoPos);
            }
        }

        // ========== TELEMETRY ==========
        telemetry.addLine("=== TURRET + DRIVE TEST ===");
        telemetry.addLine();

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Status", turret.isDriveInitialized() ? "OK" : "FAILED");
        telemetry.addData("Speed", speedMode == 1 ? "SLOW 35%" : speedMode == 2 ? "FAST 95%" : "NORMAL 65%");
        telemetry.addLine();

        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Mode", autoTrackEnabled ? "AUTO-TRACK" : "MANUAL");
        telemetry.addData("Alliance", alliance == AllianceColor.BLUE ? "BLUE (Tag 20)" : "RED (Tag 24)");
        telemetry.addData("Limelight", turret.isLimelightConnected() ? "CONNECTED" : "NOT CONNECTED");

        if (autoTrackEnabled) {
            telemetry.addData("Status", turret.getDebugState());
            telemetry.addData("Locked", turret.isLocked() ? "YES" : "no");
            telemetry.addData("TX", "%.1f°", turret.getTx());
            telemetry.addData("TY", "%.1f°", turret.getTy());
            telemetry.addData("Servo Pos", "%.3f", turret.getServoPosition());
            telemetry.addData("Tags Found", turret.getDetectedTagIds());
            telemetry.addData("Tag Count", turret.getDetectedTagCount());
        } else {
            telemetry.addData("Servo Pos", "%.3f", manualServoPos);
            telemetry.addLine("D-Pad L/R or Right Stick = Adjust");
        }

        telemetry.addLine();
        telemetry.addLine("A = Toggle auto-track | B = Alliance | Y = Reset");
        telemetry.addLine("LB = Slow | RB = Fast");
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stopDrive();
        turret.stop();
    }
}

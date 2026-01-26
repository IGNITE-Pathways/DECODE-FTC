package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.components.ActualTurretLockOn;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Test for ActualTurretLockOn - PID-based turret tracking with manual driving
 *
 * DRIVING CONTROLS:
 *   Left Stick = Drive forward/back and strafe
 *   Right Stick = Rotate
 *   LB = Toggle slow mode (even slower driving)
 *
 * TURRET CONTROLS:
 *   A = Toggle turret auto-track ON/OFF
 *   B = Switch alliance (Blue/Red tag)
 *   Y = Reset turret to center
 *   X = Increase kP by 0.001
 *   DPAD UP = Increase kP by 0.005
 *   DPAD DOWN = Decrease kP by 0.005
 *   DPAD LEFT = Decrease kD by 0.0005
 *   DPAD RIGHT = Increase kD by 0.0005
 *
 * EXPOSURE CONTROLS (Gamepad 2):
 *   RB = Toggle auto-exposure ON/OFF
 *   LT/RT = Decrease/Increase manual exposure (when auto is OFF)
 *
 * TUNING TIPS:
 *   - If turret overshoots/hunts: Increase kD (DPAD RIGHT) or decrease kP (DPAD DOWN)
 *   - If turret is too slow: Increase kP (DPAD UP)
 *   - If turret oscillates: Decrease kP and increase kD
 */
@TeleOp(name = "Test: Actual Turret PID + Drive", group = "Individual Test")
public class ActualTurretTest extends OpMode {

    // Turret
    private ActualTurretLockOn turret;

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private boolean driveInitialized = false;

    // State
    private boolean autoTrackEnabled = true;
    private AllianceColor alliance = AllianceColor.BLUE;

    // Button edge detection
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // PID tuning - Starting values (anti-overshoot tuning)
    private double currentKP = 0.008;  // Reduced to prevent overshoot
    private double currentKI = 0.0;    // Disabled to prevent oscillation
    private double currentKD = 0.005;  // Increased for damping

    // Drive settings - slow speed for precise control while turret tracks
    private static final double NORMAL_SPEED = 0.4;   // Slow speed (40%)
    private static final double SUPER_SLOW_SPEED = 0.2; // Super slow for fine positioning (20%)
    private boolean slowModeEnabled = false;
    private boolean lastLB = false;

    // Exposure controls (Gamepad 2)
    private boolean lastRB_GP2 = false;
    private boolean lastLT_GP2 = false;
    private boolean lastRT_GP2 = false;

    @Override
    public void init() {
        // Initialize turret
        turret = new ActualTurretLockOn();
        turret.initialize(hardwareMap, telemetry, alliance);

        // Initialize drive motors
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

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            driveInitialized = true;
        } catch (Exception e) {
            driveInitialized = false;
            telemetry.addData("Drive Init Error", e.getMessage());
        }

        telemetry.addLine("=== TURRET PID + DRIVE TEST ===");
        telemetry.addData("Drive", driveInitialized ? "OK" : "FAILED");
        telemetry.addLine("Sticks = Drive/Rotate | LB = Slow mode");
        telemetry.addLine("A = Track | B = Alliance | Y = Reset");
        telemetry.addLine("X = +kP | DPAD U/D = +/- kP | L/R = +/- kD");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== DRIVING (Manual, No PID) ==========
        if (driveInitialized) {
            // Get gamepad inputs
            double fwd = -gamepad1.left_stick_y;   // Forward/backward
            double str = gamepad1.left_stick_x;    // Strafe left/right
            double rot = gamepad1.right_stick_x;   // Rotate

            // LB = Toggle slow mode
            if (gamepad1.left_bumper && !lastLB) {
                slowModeEnabled = !slowModeEnabled;
            }
            lastLB = gamepad1.left_bumper;

            // Apply speed multiplier (slow driving for precision while turret tracks)
            double speedMult = slowModeEnabled ? SUPER_SLOW_SPEED : NORMAL_SPEED;

            // Mecanum drive kinematics
            double flPower = (fwd + str + rot) * speedMult;
            double frPower = (fwd - str - rot) * speedMult;
            double blPower = (fwd - str + rot) * speedMult;
            double brPower = (fwd + str - rot) * speedMult;

            // Normalize powers if any exceed 1.0
            double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                                       Math.max(Math.abs(blPower), Math.abs(brPower)));
            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }

            // Apply motor powers
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }

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
        }
        lastY = gamepad1.y;

        // ========== PID TUNING ==========

        // X = Small kP increase
        if (gamepad1.x && !lastX) {
            currentKP += 0.001;
            turret.setPIDGains(currentKP, currentKI, currentKD);
        }
        lastX = gamepad1.x;

        // DPAD UP = Increase kP
        if (gamepad1.dpad_up && !lastDpadUp) {
            currentKP += 0.005;
            turret.setPIDGains(currentKP, currentKI, currentKD);
        }
        lastDpadUp = gamepad1.dpad_up;

        // DPAD DOWN = Decrease kP
        if (gamepad1.dpad_down && !lastDpadDown) {
            currentKP = Math.max(0, currentKP - 0.005);
            turret.setPIDGains(currentKP, currentKI, currentKD);
        }
        lastDpadDown = gamepad1.dpad_down;

        // DPAD LEFT = Decrease kD
        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentKD = Math.max(0, currentKD - 0.0005);
            turret.setPIDGains(currentKP, currentKI, currentKD);
        }
        lastDpadLeft = gamepad1.dpad_left;

        // DPAD RIGHT = Increase kD
        if (gamepad1.dpad_right && !lastDpadRight) {
            currentKD += 0.0005;
            turret.setPIDGains(currentKP, currentKI, currentKD);
        }
        lastDpadRight = gamepad1.dpad_right;

        // ========== TURRET UPDATE ==========
        if (autoTrackEnabled) {
            turret.update();
        }

        // ========== EXPOSURE CONTROLS (Gamepad 2) ==========

        // RB (GP2) = Toggle auto-exposure
        if (gamepad2.right_bumper && !lastRB_GP2) {
            boolean currentAuto = turret.isAutoExposureEnabled();
            turret.setAutoExposure(!currentAuto);
        }
        lastRB_GP2 = gamepad2.right_bumper;

        // LT (GP2) = Decrease manual exposure
        if (gamepad2.left_trigger > 0.5 && !lastLT_GP2) {
            turret.decreaseExposure(5);
        }
        lastLT_GP2 = gamepad2.left_trigger > 0.5;

        // RT (GP2) = Increase manual exposure
        if (gamepad2.right_trigger > 0.5 && !lastRT_GP2) {
            turret.increaseExposure(5);
        }
        lastRT_GP2 = gamepad2.right_trigger > 0.5;

        // ========== TELEMETRY ==========
        telemetry.addLine("=== TURRET PID + DRIVE TEST ===");
        telemetry.addLine();

        // Drive status
        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Status", driveInitialized ? "ACTIVE" : "FAILED");
        telemetry.addData("Speed Mode", slowModeEnabled ? "SUPER SLOW (20%)" : "NORMAL (40%)");
        telemetry.addLine();

        // Turret status
        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Mode", autoTrackEnabled ? "AUTO-TRACK" : "MANUAL");
        telemetry.addData("Alliance", alliance == AllianceColor.BLUE ? "BLUE (Tag 20)" : "RED (Tag 24)");
        telemetry.addData("Limelight", turret.isLimelightConnected() ? "CONNECTED" : "NOT CONNECTED");
        telemetry.addData("Status", turret.getDebugState());
        telemetry.addData("Locked", turret.isLocked() ? "YES" : "no");
        telemetry.addData("Servo Pos", "%.3f", turret.getServoPosition());
        telemetry.addLine();

        // PID gains
        telemetry.addLine("--- PID GAINS ---");
        telemetry.addData("kP", "%.4f", currentKP);
        telemetry.addData("kI", "%.6f", currentKI);
        telemetry.addData("kD", "%.4f", currentKD);
        telemetry.addLine();

        // Exposure settings
        telemetry.addLine("--- EXPOSURE (GP2) ---");
        telemetry.addData("Mode", turret.isAutoExposureEnabled() ? "AUTO" : "MANUAL");
        if (!turret.isAutoExposureEnabled()) {
            telemetry.addData("Manual Value", turret.getManualExposure());
        }
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("GP1: L-Stick = Drive | R-Stick = Rotate | LB = Slow");
        telemetry.addLine("GP1: A = Track | B = Alliance | Y = Reset");
        telemetry.addLine("GP1: X = +kP | DPAD U/D = +/-kP | L/R = +/-kD");
        telemetry.addLine("GP2: RB = Toggle Auto-Exposure | LT/RT = -/+ Exposure");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop turret
        turret.stop();

        // Stop drive motors
        if (driveInitialized) {
            if (frontLeft != null) frontLeft.setPower(0);
            if (frontRight != null) frontRight.setPower(0);
            if (backLeft != null) backLeft.setPower(0);
            if (backRight != null) backRight.setPower(0);
        }
    }
}

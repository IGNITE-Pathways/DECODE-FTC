package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.ShooterConstants;

/**
 * Shooter Distance Test Program
 *
 * Tests flywheel power and hood settings at different distances.
 * Ramp is always kept UP during this test.
 *
 * CONTROLS (Gamepad 1):
 *   D-PAD UP     = Set to 2 feet distance preset
 *   D-PAD DOWN   = Set to 6 feet distance preset
 *   D-PAD LEFT   = Set to 10 feet distance preset
 *
 *   RIGHT BUMPER = Increase flywheel power (+5%)
 *   LEFT BUMPER  = Decrease flywheel power (-5%)
 *
 *   Y BUTTON     = Increase hood position (+0.05)
 *   A BUTTON     = Decrease hood position (-0.05)
 *
 *   RIGHT TRIGGER = Run intake
 *   LEFT TRIGGER  = Eject/reverse intake
 *
 *   X BUTTON     = Toggle flywheel on/off
 *   B BUTTON     = Emergency stop all
 */
@TeleOp(name = "Test: Shooter Distance", group = "Individual Test")
public class ShooterDistanceTest extends OpMode {

    // Components
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private Servo turretServo;
    private Limelight3A limelight;

    // Turret locked position
    private static final double TURRET_LOCKED_POSITION = 0.5;

    // Limelight constants for distance calculation
    private static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032; // 8 inches
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;   // Limelight 3A vertical FOV
    private static final int IMAGE_WIDTH_PIXELS = 1280;
    private static final int IMAGE_HEIGHT_PIXELS = 720;

    // Calculated distance
    private double limelightDistance = -1.0;

    // State variables
    private double flywheelPower = 0.6;  // Lower default to reduce current
    private double hoodPosition = 0.85;
    private boolean flywheelOn = false;
    private String currentPreset = "MANUAL";

    // Distance presets
    private static final double DISTANCE_1 = 2.0;   // 2 feet
    private static final double DISTANCE_2 = 6.0;   // 6 feet
    private static final double DISTANCE_3 = 10.0;  // 10 feet

    // Adjustment increments
    private static final double POWER_INCREMENT = 0.05;
    private static final double HOOD_INCREMENT = 0.05;

    // Edge detection for buttons
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevY = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevB = false;

    @Override
    public void init() {
        // Initialize launcher
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize intake/transfer
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        // Initialize turret servo and lock it
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            turretServo.setPosition(TURRET_LOCKED_POSITION);
            telemetry.addLine("Turret Servo: OK (locked at 0.5)");
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("Turret Servo: NOT FOUND");
        }

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(20);  // Lower poll rate to reduce load
            limelight.pipelineSwitch(3);  // Use pipeline 3
            limelight.start();
            telemetry.addLine("Limelight: OK (Pipeline 3)");
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("Limelight: NOT FOUND");
        }

        // Set initial values
        launcher.setPower(flywheelPower);
        launcher.setHoodPosition(hoodPosition);

        // Ramp always up
        intakeTransfer.transferUp();

        telemetry.addLine("=================================");
        telemetry.addLine("   SHOOTER DISTANCE TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("D-Pad UP    = 2ft preset");
        telemetry.addLine("D-Pad DOWN  = 6ft preset");
        telemetry.addLine("D-Pad LEFT  = 10ft preset");
        telemetry.addLine();
        telemetry.addLine("Bumpers     = Adjust power");
        telemetry.addLine("Y/A         = Adjust hood");
        telemetry.addLine("X           = Toggle flywheel");
        telemetry.addLine("Triggers    = Intake/Eject");
        telemetry.addLine();
        telemetry.addLine("Turret locked at 0.5");
        telemetry.addLine("Ramp always UP");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== DISTANCE PRESETS ==========

        // D-Pad UP = 2 feet
        if (gamepad1.dpad_up && !prevDpadUp) {
            setDistancePreset(DISTANCE_1, "2 FEET");
        }

        // D-Pad DOWN = 6 feet
        if (gamepad1.dpad_down && !prevDpadDown) {
            setDistancePreset(DISTANCE_2, "6 FEET");
        }

        // D-Pad LEFT = 10 feet
        if (gamepad1.dpad_left && !prevDpadLeft) {
            setDistancePreset(DISTANCE_3, "10 FEET");
        }

        // ========== MANUAL FLYWHEEL POWER ADJUSTMENT ==========

        // Right Bumper = Increase power
        if (gamepad1.right_bumper && !prevRightBumper) {
            flywheelPower = Math.min(1.0, flywheelPower + POWER_INCREMENT);
            launcher.setPower(flywheelPower);
            currentPreset = "MANUAL";
        }

        // Left Bumper = Decrease power
        if (gamepad1.left_bumper && !prevLeftBumper) {
            flywheelPower = Math.max(0.0, flywheelPower - POWER_INCREMENT);
            launcher.setPower(flywheelPower);
            currentPreset = "MANUAL";
        }

        // ========== MANUAL HOOD ADJUSTMENT ==========

        // Y Button = Increase hood (steeper angle)
        if (gamepad1.y && !prevY) {
            hoodPosition = Math.min(1.0, hoodPosition + HOOD_INCREMENT);
            launcher.setHoodPosition(hoodPosition);
            currentPreset = "MANUAL";
        }

        // A Button = Decrease hood (flatter angle)
        if (gamepad1.a && !prevA) {
            hoodPosition = Math.max(0.0, hoodPosition - HOOD_INCREMENT);
            launcher.setHoodPosition(hoodPosition);
            currentPreset = "MANUAL";
        }

        // ========== FLYWHEEL ON/OFF ==========

        // X Button = Toggle flywheel
        if (gamepad1.x && !prevX) {
            flywheelOn = !flywheelOn;
            launcher.setSpinning(flywheelOn);
        }

        // ========== EMERGENCY STOP ==========

        // B Button = Stop everything
        if (gamepad1.b && !prevB) {
            flywheelOn = false;
            launcher.setSpinning(false);
            intakeTransfer.stopIntake();
        }

        // ========== INTAKE CONTROL ==========

        if (gamepad1.right_trigger > 0.1) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
        }

        // ========== KEEP RAMP UP ==========
        intakeTransfer.transferUp();

        // ========== KEEP TURRET LOCKED ==========
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }

        // ========== UPDATE LAUNCHER ==========
        launcher.update();

        // ========== LIMELIGHT DISTANCE CALCULATION ==========
        limelightDistance = -1.0;
        if (limelight != null && limelight.isConnected()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double taPercent = result.getTa();
                if (taPercent > 0.0) {
                    double pixelArea = (taPercent / 100.0) * (IMAGE_WIDTH_PIXELS * IMAGE_HEIGHT_PIXELS);
                    double tagPixelHeight = Math.sqrt(pixelArea);
                    double focalPx = (IMAGE_HEIGHT_PIXELS / 2.0)
                            / Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV_DEGREES / 2.0));

                    double distanceMeters = (APRILTAG_REAL_HEIGHT_METERS * focalPx) / tagPixelHeight;
                    limelightDistance = distanceMeters * 3.28084; // Convert to feet
                }
            }
        }

        // ========== UPDATE EDGE DETECTION ==========
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        prevY = gamepad1.y;
        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevB = gamepad1.b;

        // ========== TELEMETRY ==========
        displayTelemetry();
    }

    private void setDistancePreset(double distanceFeet, String presetName) {
        currentPreset = presetName;

        // Get optimal values from ShooterConstants
        flywheelPower = ShooterConstants.getFlywheelPowerForDistance(distanceFeet);
        hoodPosition = ShooterConstants.getHoodPositionForDistance(distanceFeet);

        // Apply to launcher
        launcher.setPower(flywheelPower);
        launcher.setHoodPosition(hoodPosition);
    }

    private void displayTelemetry() {
        telemetry.addLine("=================================");
        telemetry.addLine("   SHOOTER DISTANCE TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();

        telemetry.addData("Current Preset", currentPreset);
        telemetry.addLine();

        telemetry.addLine("--- LIMELIGHT DISTANCE ---");
        if (limelightDistance > 0) {
            telemetry.addData("Distance", "%.2f ft", limelightDistance);
        } else {
            telemetry.addData("Distance", "NO TARGET");
        }
        telemetry.addLine();

        telemetry.addLine("--- FLYWHEEL ---");
        telemetry.addData("Status", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Power", "%.0f%% (LB-/RB+)", flywheelPower * 100);
        telemetry.addLine();

        telemetry.addLine("--- HOOD ---");
        telemetry.addData("Position", "%.2f (A-/Y+)", hoodPosition);
        telemetry.addLine();

        telemetry.addLine("--- INTAKE ---");
        String intakeStatus = "STOPPED";
        if (gamepad1.right_trigger > 0.1) {
            intakeStatus = "RUNNING";
        } else if (gamepad1.left_trigger > 0.1) {
            intakeStatus = "EJECTING";
        }
        telemetry.addData("Status", intakeStatus);
        telemetry.addData("Ramp", "UP (always)");
        telemetry.addLine();

        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Position", "%.2f (locked)", TURRET_LOCKED_POSITION);
        telemetry.addLine();

        telemetry.addLine("--- PRESETS ---");
        telemetry.addData("2ft (D-Up)", "Power: %.0f%%, Hood: %.2f",
            ShooterConstants.getFlywheelPowerForDistance(DISTANCE_1) * 100,
            ShooterConstants.getHoodPositionForDistance(DISTANCE_1));
        telemetry.addData("6ft (D-Down)", "Power: %.0f%%, Hood: %.2f",
            ShooterConstants.getFlywheelPowerForDistance(DISTANCE_2) * 100,
            ShooterConstants.getHoodPositionForDistance(DISTANCE_2));
        telemetry.addData("10ft (D-Left)", "Power: %.0f%%, Hood: %.2f",
            ShooterConstants.getFlywheelPowerForDistance(DISTANCE_3) * 100,
            ShooterConstants.getHoodPositionForDistance(DISTANCE_3));

        telemetry.update();
    }
}

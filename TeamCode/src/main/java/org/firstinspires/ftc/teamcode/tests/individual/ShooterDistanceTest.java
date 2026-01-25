package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;

/**
 * Shooter Distance Test Program
 *
 * Tests flywheel RPM and hood settings at different distances.
 * Uses PIDF velocity control for consistent RPM.
 * Ramp is always kept UP during this test.
 *
 * CONTROLS (Gamepad 1):
 *   D-PAD UP     = Set to 2 feet distance preset
 *   D-PAD DOWN   = Set to 6 feet distance preset
 *   D-PAD LEFT   = Set to 10 feet distance preset
 *
 *   RIGHT BUMPER = Increase flywheel RPM (+100)
 *   LEFT BUMPER  = Decrease flywheel RPM (-100)
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
    private double flywheelRPM = 3500;  // Default target RPM
    private double hoodPosition = 0.85;
    private boolean flywheelOn = false;
    private String currentPreset = "MANUAL";

    // Distance presets
    private static final double DISTANCE_1 = 2.0;   // 2 feet
    private static final double DISTANCE_2 = 6.0;   // 6 feet
    private static final double DISTANCE_3 = 10.0;  // 10 feet

    // Adjustment increments
    private static final double RPM_INCREMENT = 100;  // ±100 RPM per press
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

        // Enable PIDF velocity control
        launcher.setVelocityControlEnabled(RobotConstants.USE_VELOCITY_CONTROL);

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
        launcher.setTargetRPM(flywheelRPM);
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
        telemetry.addLine("Bumpers     = Adjust RPM (±100)");
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

        // ========== MANUAL FLYWHEEL RPM ADJUSTMENT ==========

        // Right Bumper = Increase RPM
        if (gamepad1.right_bumper && !prevRightBumper) {
            flywheelRPM = Math.min(6000, flywheelRPM + RPM_INCREMENT);
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
            }
            currentPreset = "MANUAL";
        }

        // Left Bumper = Decrease RPM
        if (gamepad1.left_bumper && !prevLeftBumper) {
            flywheelRPM = Math.max(1000, flywheelRPM - RPM_INCREMENT);
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
            }
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

        // Get optimal values from RobotConstants based on distance ranges
        if (distanceFeet >= RobotConstants.RANGE_1_MIN && distanceFeet <= RobotConstants.RANGE_1_MAX) {
            // Range 1: 2.45 - 3.45 ft
            flywheelRPM = RobotConstants.RANGE_1_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_1_HOOD_POSITION;
        } else if (distanceFeet >= RobotConstants.RANGE_2_MIN && distanceFeet <= RobotConstants.RANGE_2_MAX) {
            // Range 2: 3.46 - 4.65 ft
            flywheelRPM = RobotConstants.RANGE_2_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_2_HOOD_POSITION;
        } else if (distanceFeet >= RobotConstants.RANGE_3_MIN && distanceFeet <= RobotConstants.RANGE_3_MAX) {
            // Range 3: 4.66 - 5.35 ft
            flywheelRPM = RobotConstants.RANGE_3_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_3_HOOD_POSITION;
        } else if (distanceFeet >= RobotConstants.RANGE_4_MIN && distanceFeet <= RobotConstants.RANGE_4_MAX) {
            // Range 4: 5.36 - 6.00 ft
            flywheelRPM = RobotConstants.RANGE_4_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_4_HOOD_POSITION;
        } else if (distanceFeet >= RobotConstants.RANGE_FAR_MIN) {
            // Far range: 10+ ft
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
        } else {
            // Default values for out of range
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
        }

        // Apply to launcher (both when flywheel is on or off - values are stored)
        launcher.setTargetRPM(flywheelRPM);
        launcher.setHoodPosition(hoodPosition);
    }

    private double getRPMForDistance(double distanceFeet) {
        // Return RPM based on distance ranges (for telemetry display)
        if (distanceFeet >= RobotConstants.RANGE_1_MIN && distanceFeet <= RobotConstants.RANGE_1_MAX) {
            return RobotConstants.RANGE_1_FLYWHEEL_RPM;
        } else if (distanceFeet >= RobotConstants.RANGE_2_MIN && distanceFeet <= RobotConstants.RANGE_2_MAX) {
            return RobotConstants.RANGE_2_FLYWHEEL_RPM;
        } else if (distanceFeet >= RobotConstants.RANGE_3_MIN && distanceFeet <= RobotConstants.RANGE_3_MAX) {
            return RobotConstants.RANGE_3_FLYWHEEL_RPM;
        } else if (distanceFeet >= RobotConstants.RANGE_4_MIN && distanceFeet <= RobotConstants.RANGE_4_MAX) {
            return RobotConstants.RANGE_4_FLYWHEEL_RPM;
        } else if (distanceFeet >= RobotConstants.RANGE_FAR_MIN) {
            return RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
        } else {
            return RobotConstants.DEFAULT_TARGET_RPM;
        }
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
        telemetry.addData("Target RPM", "%.0f RPM (LB-/RB+)", flywheelRPM);
        if (flywheelOn) {
            telemetry.addData("Actual RPM", "%.0f RPM", launcher.getCurrentRPM());
        }
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
        telemetry.addData("2ft (D-Up)", "RPM: %.0f, Hood: %.2f",
            getRPMForDistance(DISTANCE_1),
            RobotConstants.getHoodPositionForDistance(DISTANCE_1));
        telemetry.addData("6ft (D-Down)", "RPM: %.0f, Hood: %.2f",
            getRPMForDistance(DISTANCE_2),
            RobotConstants.getHoodPositionForDistance(DISTANCE_2));
        telemetry.addData("10ft (D-Left)", "RPM: %.0f, Hood: %.2f",
            getRPMForDistance(DISTANCE_3),
            RobotConstants.getHoodPositionForDistance(DISTANCE_3));

        telemetry.update();
    }
}

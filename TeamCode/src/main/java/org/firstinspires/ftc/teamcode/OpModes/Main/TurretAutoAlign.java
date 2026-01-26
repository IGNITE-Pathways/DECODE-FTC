package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name="Turret Smart Lock v2 (Alliance Switch)", group="Main")
public class TurretAutoAlign extends OpMode {

    private Limelight3A limelight;
    private Servo turretServo;

    // PID constants
    private double kP = 0.00035;
    private double kI = 0.0;
    private double kD = 0.00012;

    private double integral = 0;
    private double lastError = 0;

    // Servo config
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double TARGET_TX = 11.5;  // ideal tx
    private static final double IDEAL_RANGE = 0.05; // tight tolerance
    private static final double INTEGRAL_WINDUP_LIMIT = 100.0; // Prevent integral windup
    private boolean CHECK = false; // Instance variable, not static
    private double servoPos = 0.5;
    private boolean aligned = false;
    private boolean appliedLeadOffset = false;
    private boolean lastFrameLocked = false;

    // Direction memory
    private int lastDirection = 1; // 1 = right, -1 = left

    // Scanning speed and behavior
    private double scanSpeed = 0.0008;
    private double searchRange = 0.05; // how far to scan around last seen position
    private int framesSinceSeen = 0;   // expands search zone over time if lost
    private double lockedServoPos = 0.5;

    // Max incremental adjustment
    private static final double MAX_OUTPUT = 0.0005;

    // Servo direction multiplier
    private double servoDirection = 1.0;

    // Alliance color tracking
    private static final int BLUE_TAG = 20;
    private static final int RED_TAG = 24;
    private int targetTagId = RED_TAG;  // Default to RED (Tag 24)
    private String currentAlliance = "RED";

    // Limelight pipeline
    private static final int APRILTAG_PIPELINE = 3;  // Pipeline 3 for AprilTag detection

    // Button states for alliance switching
    private boolean lastLB = false;
    private boolean lastRB = false;

    @Override
    public void init() {
        try {
            // Initialize servo
            turretServo = hardwareMap.get(Servo.class, "turretServo");
            turretServo.setPosition(servoPos);

            // Initialize limelight with AprilTag pipeline
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(APRILTAG_PIPELINE);  // Switch to AprilTag pipeline (Pipeline 3)
                limelight.setPollRateHz(100);
                limelight.start();

                telemetry.addLine("✅ Turret Smart Lock Initialized");
                telemetry.addData("Limelight", "CONNECTED - Pipeline " + APRILTAG_PIPELINE);
            } else {
                telemetry.addLine("❌ Limelight: NULL");
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
        }

        telemetry.addData("Alliance", currentAlliance + " (Tag " + targetTagId + ")");
        telemetry.addLine();
        telemetry.addLine("LB = Switch to BLUE (Tag 20)");
        telemetry.addLine("RB = Switch to RED (Tag 24)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== ALLIANCE COLOR SWITCHING ==========
        // LB = Switch to BLUE alliance (Tag 20)
        if (gamepad1.left_bumper && !lastLB) {
            targetTagId = BLUE_TAG;
            currentAlliance = "BLUE";
            resetTracking();
        }
        lastLB = gamepad1.left_bumper;

        // RB = Switch to RED alliance (Tag 24)
        if (gamepad1.right_bumper && !lastRB) {
            targetTagId = RED_TAG;
            currentAlliance = "RED";
            resetTracking();
        }
        lastRB = gamepad1.right_bumper;

        // ========== LIMELIGHT CHECK ==========
        if (limelight == null) {
            telemetry.addLine("❌ Limelight: NULL (not found in hardware map)");
            telemetry.addLine("Check hardware config: 'limelight'");
            telemetry.addData("Alliance", currentAlliance);
            telemetry.update();
            return;
        }

        if (!limelight.isConnected()) {
            telemetry.addLine("❌ Limelight: NOT CONNECTED");
            telemetry.addLine("Check USB cable and power");
            telemetry.addData("Alliance", currentAlliance);
            telemetry.update();
            return;
        }

        LLResult result = limelight.getLatestResult();
        double tx = 0;
        boolean targetTagDetected = false;
        int detectedTagCount = 0;
        String detectedTagIds = "none";

        // Detect only the target tag (based on current alliance)
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (fiducialResults != null && !fiducialResults.isEmpty()) {
                detectedTagCount = fiducialResults.size();
                StringBuilder ids = new StringBuilder();

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (ids.length() > 0) ids.append(", ");
                    ids.append(fr.getFiducialId());

                    if (fr.getFiducialId() == targetTagId) {
                        tx = fr.getTargetXDegrees();
                        targetTagDetected = true;
                    }
                }
                detectedTagIds = ids.toString();
            }
        }

        if (targetTagDetected) {
            framesSinceSeen = 0;
            double error = tx - TARGET_TX;

            // ---------------- LOCK BEHAVIOR ----------------
            if (Math.abs(error) <= IDEAL_RANGE || CHECK) {
                aligned = true;
                servoPos = servoPos; // hold current position
                turretServo.setPosition(servoPos);

                integral = 0;
                lastError = 0;
                lastFrameLocked = true;

                telemetry.addLine("🎯 Aligned! Holding position");
                telemetry.addData("Alliance", currentAlliance + " (Tag " + targetTagId + ")");
                telemetry.addData("Tags Detected", detectedTagCount + " (" + detectedTagIds + ")");
                telemetry.addData("tx", "%.2f°", tx);
                telemetry.addData("Error", "%.2f°", error);
                telemetry.addData("Servo Pos", "%.3f", servoPos);

            } else {
                // ---------------- PID ACTIVE ----------------
                aligned = false;
                appliedLeadOffset = false;

                integral += error;
                double derivative = error - lastError;
                double rawOutput = (kP * error) + (kI * integral) + (kD * derivative);
                double output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, rawOutput));


                // Move servo toward reducing error
                double step = Math.signum(output) * Math.min(MAX_OUTPUT, Math.abs(output)); // incremental step
                servoPos += servoDirection * step;
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(servoPos);
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(servoPos);




                lastDirection = (servoDirection * output) >= 0 ? 1 : -1;
                lastError = error;
                lastFrameLocked = true;

                telemetry.addLine("⚡ PID Active: Correcting Position");
                telemetry.addData("Alliance", currentAlliance + " (Tag " + targetTagId + ")");
                telemetry.addData("Tags Detected", detectedTagCount + " (" + detectedTagIds + ")");
                telemetry.addData("tx", "%.2f°", tx);
                telemetry.addData("Error", "%.2f°", error);
                telemetry.addData("Servo Pos", "%.3f", servoPos);
                servoPos = servoPos + (error * -0.01);
                turretServo.setPosition(servoPos);
                CHECK = true;
            }

            lockedServoPos = servoPos;

        } else {
            // ---------------- TAG LOST ----------------
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

            telemetry.addLine("🔍 No Target Tag — Smart Scanning");
            telemetry.addData("Alliance", currentAlliance + " (Tag " + targetTagId + ")");
            telemetry.addData("Tags Detected", detectedTagCount + " (" + detectedTagIds + ")");
            telemetry.addData("Servo Pos", "%.3f", servoPos);
            telemetry.addData("Search Range", "%.2f", dynamicRange);
            telemetry.addData("Scan Direction", lastDirection == 1 ? "Right" : "Left");
            lastFrameLocked = false;
        }

        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("LB = BLUE (Tag 20) | RB = RED (Tag 24)");
        telemetry.update();
    }

    /**
     * Reset tracking state when switching alliances
     */
    private void resetTracking() {
        integral = 0;
        lastError = 0;
        aligned = false;
        appliedLeadOffset = false;
        lastFrameLocked = false;
        CHECK = false;
        framesSinceSeen = 0;
        // Keep current servo position to avoid jumps
    }
}

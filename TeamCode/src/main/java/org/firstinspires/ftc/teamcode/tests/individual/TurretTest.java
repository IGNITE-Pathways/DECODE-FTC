package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.components.Turret;
@Disabled
@TeleOp(name="Test: Turret", group="Individual Test")
public class TurretTest extends OpMode {

    private Turret turret;
    private Servo turretServo;
    private Limelight3A limelight;

    // Manual control mode
    private boolean manualMode = true; // Start in manual mode
    private double servoPosition = 0.5; // Initial position (0.375 to 0.8 range)
    private static final double SERVO_MIN = 0.375;
    private static final double SERVO_MAX = 0.8;
    private static final double INCREMENT = 0.01; // Small increments for precise control
    private static final double STICK_SENSITIVITY = 0.003; // How fast the stick moves the servo

    // Edge detection for mode toggle
    private boolean prevLeftBumper = false;

    @Override
    public void init() {
        // Get hardware directly for manual control
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize turret component for auto-mode
        turret = new Turret();
        turret.initialize(hardwareMap, telemetry);

        // Set initial servo position
        if (turretServo != null) {
            turretServo.setPosition(servoPosition);
        }

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }

        telemetry.addLine("Turret Test Initialized");
        telemetry.addLine("=== MANUAL CONTROLS ===");
        telemetry.addLine("Right Stick X: Smooth movement");
        telemetry.addLine("D-Pad Left/Right: Fine adjustments");
        telemetry.addLine("Left Bumper: Toggle Manual/Auto mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle between manual and auto mode
        if (gamepad1.left_bumper && !prevLeftBumper) {
            manualMode = !manualMode;
        }
        prevLeftBumper = gamepad1.left_bumper;

        if (manualMode) {
            // Manual control mode
            // Right Stick X: Smooth analog control
            double stickInput = gamepad1.right_stick_x;
            if (Math.abs(stickInput) > 0.1) { // Dead zone
                servoPosition += stickInput * STICK_SENSITIVITY;
            }

            // D-Pad Left: Fine adjustment left (decrease position)
            if (gamepad1.dpad_left) {
                servoPosition -= INCREMENT;
            }

            // D-Pad Right: Fine adjustment right (increase position)
            if (gamepad1.dpad_right) {
                servoPosition += INCREMENT;
            }

            // Clamp servo position within valid range
            servoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPosition));

            // Apply position
            if (turretServo != null) {
                turretServo.setPosition(servoPosition);
            }

            // Telemetry for manual mode
            telemetry.addLine("=== MANUAL MODE ===");
            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.addData("Range", "%.3f - %.3f", SERVO_MIN, SERVO_MAX);
            telemetry.addData("Right Stick X", "%.2f", stickInput);
            telemetry.addLine("Right Stick X: Smooth movement");
            telemetry.addLine("D-Pad Left/Right: Fine adjustments");

        } else {
            // Auto-alignment mode
            boolean limelightConnected = turret.update();

            if (!limelightConnected) {
                telemetry.addLine("=== AUTO MODE ===");
                telemetry.addLine("⚠️ Limelight not connected");
                telemetry.addLine("Check hardware configuration");
            } else {
                telemetry.addLine("=== AUTO MODE ===");
                telemetry.addLine("✅ Limelight connected");
                telemetry.addLine("Turret is auto-aligning with AprilTag");

                // Display limelight data
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("ta", result.getTa());
                }
            }
        }

        telemetry.addLine("");
        telemetry.addData("Mode", manualMode ? "MANUAL" : "AUTO");
        telemetry.addLine("Left Bumper: Toggle Mode");
        telemetry.update();
    }
}
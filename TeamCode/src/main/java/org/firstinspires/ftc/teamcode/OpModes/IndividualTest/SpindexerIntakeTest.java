package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Intake;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Spindexer;

@TeleOp(name = "Test: Spindexer + Intake", group = "Test")
public class SpindexerIntakeTest extends LinearOpMode {

    private Intake intake;
    private Spindexer spindexer;

    // Ball order tracking (for use in both TeleOp and AutoOp)
    private String[] ballOrder = new String[3];  // Max 3 balls: PPG, PGP, or GPP
    private int ballCount = 0;  // Current number of balls tracked (0-3)
    // Button state tracking for edge detection
    private boolean prevRightBumper = false;
    private boolean prevRightTrigger = false;

    @Override
    public void runOpMode() {
        // Initialize components
        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        telemetry.addLine("Spindexer + Intake Test Initialized");
        telemetry.addLine("Right Bumper: Start Intake + Color Sensing");
        telemetry.addLine("Right Trigger: Stop Intake + Color Sensing");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read gamepad inputs for intake
            boolean rightBumper = gamepad1.right_bumper;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;  // Threshold for trigger press
         
            // Handle intake start/stop controls
            if (rightBumper && !prevRightBumper) {
                // Start intake and color sensing
                intake.start();
                spindexer.startSensing(this::onBallDetected);
                telemetry.addLine("Intake started, color sensing active");
            }
            prevRightBumper = rightBumper;
            
            if (rightTrigger && !prevRightTrigger) {
                // Stop intake and color sensing
                intake.stop();
                spindexer.stopSensing();
                telemetry.addLine("Intake stopped, color sensing stopped");
            }
            prevRightTrigger = rightTrigger;
            
            // Update color sensing (needs to be called every loop iteration when active)
            spindexer.updateSensing();
            
            // Update intake (handles power ramping if enabled)
            intake.update();
        
            addIntakeTelemetry();
            
            telemetry.update();

            idle();
        }
    }

    private void onBallDetected(String color) {
        if (ballCount < 3) {
            ballOrder[ballCount] = color;
            ballCount++;
        }
        // Rotate spindexer by one cycle when ball is detected
        spindexer.rotateOneDivision();
    }

    private void addIntakeTelemetry() {
        // Add intake-specific telemetry
        telemetry.addLine("");
        telemetry.addLine("=== INTAKE STATUS ===");
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Intake Target", "%.2f", intake.getTargetPower());
        telemetry.addData("Intake Status", intake.isRunning() ? "Running" : "Stopped");
        telemetry.addData("Intake State", intake.getState().toString());
        if (intake.isRampingEnabled()) {
            telemetry.addData("Intake Ramping", "Yes (%.3f/update)", intake.getRampRate());
        }
        
        telemetry.addLine("");
        telemetry.addLine("=== ADDITIONAL SPINDEXER INFO ===");
        telemetry.addData("Ball Count", spindexer.getBallCount());
        telemetry.addData("All Balls Intaked", spindexer.areAllBallsIntaked() ? "Yes" : "No");
        telemetry.addData("Color Sensing Active", spindexer.isSensing() ? "Yes" : "No");
        
        telemetry.addLine("");
        telemetry.addLine("=== BALL ORDER TRACKING ===");
        telemetry.addData("Tracked Ball Count", ballCount);
        if (ballCount > 0) {
            StringBuilder order = new StringBuilder();
            for (int i = 0; i < ballCount; i++) {
                if (i > 0) order.append("-");
                order.append(ballOrder[i] != null ? ballOrder[i] : "null");
            }
            telemetry.addData("Ball Order", order.toString());
        } else {
            telemetry.addData("Ball Order", "None");
        }
    }
}


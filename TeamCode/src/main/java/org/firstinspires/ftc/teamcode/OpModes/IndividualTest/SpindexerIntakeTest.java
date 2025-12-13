package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Intake;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Spindexer;

import java.util.Map;

@TeleOp(name = "Test: Spindexer + Intake", group = "Test")
public class SpindexerIntakeTest extends LinearOpMode {

    private Intake intake;
    private Spindexer spindexer;

    // Button state tracking for edge detection


    @Override
    public void runOpMode() {
        // Initialize components
        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        telemetry.addLine("Spindexer + Intake Test Initialized");

        telemetry.addLine("=== Intake Controls (A, B, X, Y) ===");
        telemetry.addLine("A: Start intake");
        telemetry.addLine("B: Stop intake");
        telemetry.addLine("X: Reverse intake (eject)");
        telemetry.addLine("Y: Toggle spindexer intake servo");
        telemetry.addLine("=== Spindexer Controls ===");
        telemetry.addLine("Right Bumper: Rotate spindexer one division");
        telemetry.addLine("Right Trigger: Reset spindexer to division 0");
        telemetry.addLine("Left Trigger: Shoot ball");
        telemetry.addLine("Left Bumper: Toggle rapid fire/indexing mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read gamepad inputs
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftTrigger = gamepad1.left_trigger > 0.5;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;

            // ========== INTAKE CONTROLS (A, B, X, Y) ==========

            // Y: Toggle spindexer intake servo (handled by spindexer.update)
            // We'll pass false for A, B, X to spindexer since we're using them for intake
            // and use triggers/bumpers for spindexer controls

            // ========== SPINDEXER CONTROLS (Bumpers/Triggers) ==========
            
            // Map spindexer controls to new buttons:
            // Right Bumper = A (rotate)
            // Right Trigger = B (reset)
            // Left Trigger = X (shoot)
            // Y = Y (toggle intake servo)
            // Left Bumper = Left Bumper (toggle mode)
            boolean spindexerA = rightBumper;
            boolean spindexerB = rightTrigger;
            boolean spindexerX = leftTrigger;
            
            // Update spindexer with mapped inputs
            spindexer.update(spindexerA, spindexerB, spindexerX, y, leftBumper);



            // ========== UPDATE COMPONENTS ==========
            
            // Update intake (handles power ramping, telemetry, etc.)
            intake.update();
            
            // Update spindexer (this also adds telemetry via addTelemetry() call)
            // spindexer.update() is already called above with gamepad inputs
            
            // ========== TELEMETRY DISPLAY ==========
            
            // Add intake telemetry (spindexer telemetry is already added by spindexer.update())
            addIntakeTelemetry();
            
            telemetry.update();

            idle();
        }
    }

    private void addIntakeTelemetry() {
        // Add intake-specific telemetry
        // Note: spindexer.update() already calls addTelemetry() which shows spindexer info
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
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Start Intake | B: Stop Intake | X: Reverse Intake");
        telemetry.addLine("Y: Toggle Spindexer Intake | Right Bumper: Rotate | Right Trigger: Reset | Left Trigger: Shoot");
        telemetry.addLine("Left Bumper: Toggle Rapid Fire/Indexing Mode");
    }
}


package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Intake;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Spindexer;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Test: Spindexer + Intake", group = "Test")
public class SpindexerIntakeTest extends LinearOpMode {

    private Intake intake;
    private Spindexer spindexer;

    // Slot constants (matching SpindexerOLD.java)
    private static final int INTAKE_SLOT = 0;   // Fixed intake position
    private static final int LAUNCH_SLOT = 1;   // Fixed launch position
    private static final int MIDDLE_SLOT = 2;   // Fixed middle position

    // Ball slot tracking - tracks which ball is in which slot position
    private Map<Integer, String> ballSlots = new HashMap<>();
    
    private int ballCount = 0;  // Current number of balls detected (0-3)
    // Button state tracking for edge detection
    private boolean prevRightBumper = false;
    private boolean prevRightTrigger = false;
    private boolean prevA = false;
    private boolean prevB = false;

    @Override
    public void runOpMode() {
        // Initialize components
        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        // Initialize ball slot tracking
        ballSlots.put(INTAKE_SLOT, "none");
        ballSlots.put(MIDDLE_SLOT, "none");
        ballSlots.put(LAUNCH_SLOT, "none");

        telemetry.addLine("Spindexer + Intake Test Initialized");
        telemetry.addLine("Right Bumper: Start Intake + Color Sensing");
        telemetry.addLine("Right Trigger: Stop Intake + Color Sensing");
        telemetry.addLine("A: Rotate One Division");
        telemetry.addLine("B: Kick");

        telemetry.update();

        waitForStart();
        
        while (opModeIsActive()) {
            // Read gamepad inputs for intake
            boolean rightBumper = gamepad1.right_bumper;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;  // Threshold for trigger press
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
         
            // Handle intake start/stop controls
            if (rightBumper && !prevRightBumper) {
                // Start intake and color sensing
                intake.start();
                spindexer.startSensing(this::onBallDetected);
                telemetry.addLine("Intake started, color sensing active");
            }
            prevRightBumper = rightBumper;
            
            if ((rightTrigger && !prevRightTrigger) || ballCount >= 3) {
                // Stop intake and color sensing
                intake.stop();
                spindexer.stopSensing();
                telemetry.addLine("Intake stopped, color sensing stopped");
            }
            prevRightTrigger = rightTrigger;
            
            // Handle spindexer controls
            if (a && !prevA) {
                spindexer.rotateOneDivision();
                // Shift balls between slots when rotating
                shiftBallsBetweenSlots();
            }
            prevA = a;
            
            if (b && !prevB) {
                spindexer.kick();
                // Clear launch slot after kick
                ballSlots.put(LAUNCH_SLOT, "none");
            }
            prevB = b;
            
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
            ballCount++;
            // Assign detected ball to INTAKE_SLOT (slot 0)
            ballSlots.put(INTAKE_SLOT, color);
        }
        // Rotate spindexer by one cycle when ball is detected
        spindexer.rotateOneDivision();
        // Shift balls between slots after rotation
        shiftBallsBetweenSlots();
    }

    /**
     * Shifts balls between slots when spindexer rotates one division.
     * Rotation pattern (clockwise): INTAKE_SLOT -> MIDDLE_SLOT -> LAUNCH_SLOT -> INTAKE_SLOT
     * Matches SpindexerOLD.java rotation logic
     */
    private void shiftBallsBetweenSlots() {
        // Save current state
        String launchSlotBall = ballSlots.get(LAUNCH_SLOT);
        String middleSlotBall = ballSlots.get(MIDDLE_SLOT);
        String intakeSlotBall = ballSlots.get(INTAKE_SLOT);

        // Rotate clockwise: LAUNCH -> INTAKE, INTAKE -> MIDDLE, MIDDLE -> LAUNCH
        ballSlots.put(INTAKE_SLOT, launchSlotBall);
        ballSlots.put(MIDDLE_SLOT, intakeSlotBall);
        ballSlots.put(LAUNCH_SLOT, middleSlotBall);
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
        telemetry.addLine("=== SLOT POSITIONS ===");
        telemetry.addData("Balls Detected", ballCount);
        String intakeSlotColor = ballSlots.get(INTAKE_SLOT);
        String intakeDisplay = intakeSlotColor.equals("none") ? "EMPTY" : "●" + intakeSlotColor.toUpperCase();
        telemetry.addData("Slot 0 (INTAKE)", intakeDisplay);
        
        String launchSlotColor = ballSlots.get(LAUNCH_SLOT);
        String launchDisplay = launchSlotColor.equals("none") ? "EMPTY" : "●" + launchSlotColor.toUpperCase();
        telemetry.addData("Slot 1 (LAUNCH)", launchDisplay);
        
        String middleSlotColor = ballSlots.get(MIDDLE_SLOT);
        String middleDisplay = middleSlotColor.equals("none") ? "EMPTY" : "●" + middleSlotColor.toUpperCase();
        telemetry.addData("Slot 2 (MIDDLE)", middleDisplay);
    }
}


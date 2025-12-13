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
    private static final int LAST_SLOT = 2;   // Fixed middle position

    // Desired launch order: PPG (Purple, Purple, Green)
    private static final String[] DESIRED_LAUNCH_ORDER = {"purple", "purple", "green"};

    // Ball slot tracking - tracks which ball is in which slot position
    private Map<Integer, String> ballSlots = new HashMap<>();
    
    private int ballCount = 0;  // Current number of balls detected (0-3)
    private int launchIndex = 0;  // Current index in DESIRED_LAUNCH_ORDER (0-2)
    // Button state tracking for edge detection
    private boolean prevRightBumper = false;
    private boolean prevRightTrigger = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {
        // Initialize components
        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        // Initialize ball slot tracking
        ballSlots.put(INTAKE_SLOT, "none");
        ballSlots.put(LAST_SLOT, "none");
        ballSlots.put(LAUNCH_SLOT, "none");

        telemetry.addLine("Spindexer + Intake Test Initialized");
        telemetry.addLine("Right Bumper: Start Intake + Color Sensing");
        telemetry.addLine("Right Trigger: Stop Intake + Color Sensing");
        telemetry.addLine("A: Rotate One Division");
        telemetry.addLine("B: Kick");
        telemetry.addLine("X: Pseudo Launch (PPG order)");

        telemetry.update();

        waitForStart();
        
        while (opModeIsActive()) {
            // Read gamepad inputs for intake
            boolean rightBumper = gamepad1.right_bumper;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;  // Threshold for trigger press
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
         
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
                
                // Reset launch index when we have 3 balls loaded via Intake
                if (getCurrentBallCount() >= 3) {
                    launchIndex = 0;
                    telemetry.addLine("Launch sequence reset - 3 balls loaded!");
                }
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
            
            if (x && !prevX) {
                pseudoLaunch();
            }
            prevX = x;
            
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
            ballSlots.put(LAST_SLOT, color);
        }
        // Rotate spindexer by one cycle when ball is detected
        spindexer.rotateOneDivision();
        // Shift balls between slots after rotation
        shiftBallsBetweenSlots();
        
        // Reset launch index when we have 3 balls again
        if (ballCount >= 3) {
            launchIndex = 0;
        }
    }

    /**
     * Shifts balls between slots when spindexer rotates one division.
     * Rotation pattern (clockwise): INTAKE_SLOT -> LAST_SLOT -> LAUNCH_SLOT -> INTAKE_SLOT
     * Matches SpindexerOLD.java rotation logic
     */
    private void shiftBallsBetweenSlots() {
        // Save current state
        String launchSlotBall = ballSlots.get(LAUNCH_SLOT);
        String lastSlotBall = ballSlots.get(LAST_SLOT);
        String intakeSlotBall = ballSlots.get(INTAKE_SLOT);

        // Rotate clockwise: LAUNCH -> INTAKE, INTAKE -> MIDDLE, MIDDLE -> LAUNCH
        ballSlots.put(INTAKE_SLOT, lastSlotBall);
        ballSlots.put(LAUNCH_SLOT, intakeSlotBall);
        ballSlots.put(LAST_SLOT, launchSlotBall);
    }

    /**
     * Pseudo launch method that launches one ball at a time in the desired launch order (PPG).
     * Each press of X launches the next ball in sequence.
     * Rotates the spindexer to ensure the correct color ball is in the launch slot, then clears it.
     */
    private void pseudoLaunch() {
        // Check if we have 3 balls loaded
        int currentBallCount = getCurrentBallCount();
        
        if (currentBallCount < 3) {
            telemetry.addLine("Cannot launch: Need 3 balls loaded. Current: " + currentBallCount);
            telemetry.update();
            return;
        }
        
        // Check if we've already launched all balls
        if (launchIndex >= DESIRED_LAUNCH_ORDER.length) {
            telemetry.addLine("All balls launched! Load 3 more balls using Intake to launch again.");
            telemetry.update();
            return;
        }
        
        // Get the desired color for this launch
        String desiredColor = DESIRED_LAUNCH_ORDER[launchIndex];
        telemetry.addLine("Launching ball " + (launchIndex + 1) + "/3: " + desiredColor);
        telemetry.update();
        
        // Find which slot contains the desired color ball
        int ballSlot = findSlotWithColor(desiredColor);
        
        if (ballSlot == -1) {
            telemetry.addLine("WARNING: " + desiredColor + " ball not found! Skipping...");
            telemetry.update();
            launchIndex++; // Still increment to avoid getting stuck
            return;
        }
        
        // Rotate spindexer to move the ball to LAUNCH_SLOT
        rotateBallToLaunchSlot(ballSlot);
        
        // Simulate launch by clearing the launch slot
        ballSlots.put(LAUNCH_SLOT, "none");
        telemetry.addLine("Launched " + desiredColor + " ball!");
        telemetry.update();
        
        // Move to next ball in launch order
        launchIndex++;
    }
    
    /**
     * Counts how many balls are currently in the slots (not "none").
     * @return Number of balls currently loaded (0-3)
     */
    private int getCurrentBallCount() {
        int count = 0;
        for (int slot : new int[]{INTAKE_SLOT, LAST_SLOT, LAUNCH_SLOT}) {
            String slotColor = ballSlots.get(slot);
            if (slotColor != null && !slotColor.equals("none")) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Finds which slot contains a ball of the specified color.
     * @param color The color to search for ("purple" or "green")
     * @return The slot number (INTAKE_SLOT, LAST_SLOT, or LAUNCH_SLOT), or -1 if not found
     */
    private int findSlotWithColor(String color) {
        for (int slot : new int[]{LAUNCH_SLOT, INTAKE_SLOT, LAST_SLOT}) {
            String slotColor = ballSlots.get(slot);
            if (slotColor != null && slotColor.equals(color)) {
                return slot;
            }
        }
        return -1;
    }
    
    /**
     * Rotates the spindexer to move a ball from the specified slot to LAUNCH_SLOT.
     * Rotation pattern: INTAKE_SLOT -> LAST_SLOT -> LAUNCH_SLOT -> INTAKE_SLOT (clockwise)
     * 
     * @param sourceSlot The slot containing the ball to move (INTAKE_SLOT, LAST_SLOT, or LAUNCH_SLOT)
     */
    private void rotateBallToLaunchSlot(int sourceSlot) {
        if (sourceSlot == LAUNCH_SLOT) {
            // Ball is already in launch slot, no rotation needed
            return;
        }
        
        int rotationsNeeded;
        if (sourceSlot == INTAKE_SLOT) {
            // From INTAKE_SLOT to LAUNCH_SLOT: need 2 rotations
            // INTAKE -> LAST -> LAUNCH
            rotationsNeeded = 2;
        } else if (sourceSlot == LAST_SLOT) {
            // From LAST_SLOT to LAUNCH_SLOT: need 1 rotation
            // LAST -> LAUNCH
            rotationsNeeded = 1;
        } else {
            // Unknown slot, no rotation
            return;
        }
        
        // Perform the rotations
        for (int i = 0; i < rotationsNeeded; i++) {
            spindexer.rotateOneDivision();
            shiftBallsBetweenSlots();
            sleep(200); // Small delay between rotations for smooth movement
        }
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
        telemetry.addData("Current Balls Loaded", getCurrentBallCount());
        telemetry.addData("Launch Index", launchIndex + "/" + DESIRED_LAUNCH_ORDER.length);
        String intakeSlotColor = ballSlots.get(INTAKE_SLOT);
        String intakeDisplay = intakeSlotColor.equals("none") ? "EMPTY" : "●" + intakeSlotColor.toUpperCase();
        telemetry.addData("Slot 0 (INTAKE)", intakeDisplay);
        
        String launchSlotColor = ballSlots.get(LAUNCH_SLOT);
        String launchDisplay = launchSlotColor.equals("none") ? "EMPTY" : "●" + launchSlotColor.toUpperCase();
        telemetry.addData("Slot 1 (LAUNCH)", launchDisplay);
        
        String lastSlotColor = ballSlots.get(LAST_SLOT);
        String middleDisplay = lastSlotColor.equals("none") ? "EMPTY" : "●" + lastSlotColor.toUpperCase();
        telemetry.addData("Slot 2 (MIDDLE)", middleDisplay);
    }
}


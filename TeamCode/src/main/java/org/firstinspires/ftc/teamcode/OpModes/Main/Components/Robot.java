package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Robot {
    // All components
    private Turret turret;
    private Launcher launcher;
    private Spindexer spindexer;
    private DriveTrain driveTrain;
    private Lift lift;
    private Intake intake;
    
    // Ball order tracking (for use in both TeleOp and AutoOp)
    private String[] ballOrder = new String[3];  // Max 3 balls: PPG, PGP, or GPP
    private int ballCount = 0;  // Current number of balls tracked (0-3)
<<<<<<< Updated upstream
    
    // Ball slot tracking constants
    private static final int INTAKE_SLOT = 0;   // Fixed intake position
    private static final int LAUNCH_SLOT = 1;   // Fixed launch position
    private static final int LAST_SLOT = 2;     // Fixed middle position
    
    // Ball slot tracking - tracks which ball is in which slot position
    private Map<Integer, String> ballSlots = new HashMap<>();
    private int launchIndex = 0;  // Current index in detectedBallSequence (0-2)
    private float lastHueDetected = -1f;  // Hue value of the last detected ball
    
    // Obelisk detection - detected ball sequence from AprilTag
    private List<String> detectedBallSequence = null;  // null = not yet detected, populated = detected
    private int lastDetectedObeliskTagId = -1;  // Track last detected tag to prevent duplicates
    
    // AprilTag ID to ball sequence mapping (obelisk tags)
    private static final int APRILTAG_ID_PPG = 23;  // Purple - Purple - Green
    private static final int APRILTAG_ID_PGP = 22;  // Purple - Green - Purple
    private static final int APRILTAG_ID_GPP = 21;  // Green - Purple - Purple
=======
>>>>>>> Stashed changes

    /**
     * Initialize all robot components
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     * @param opMode LinearOpMode instance (needed for Spindexer)
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        initialize(hardwareMap, telemetry, opMode, null);
    }
    
    /**
     * Initialize all robot components with alliance color
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     * @param opMode LinearOpMode instance (needed for Spindexer)
     * @param allianceColor Alliance color (BLUE or RED) for turret tag detection
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, AllianceColor allianceColor) {
        // Initialize all components
        turret = new Turret();
        turret.initialize(hardwareMap, telemetry, allianceColor);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, opMode);

        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        lift = new Lift();
        lift.initialize(hardwareMap, telemetry);

        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);
        
        // Initialize ball order tracking
        resetBallTracking();

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    // Component update methods
    public void updateLauncher() {
        launcher.update();
    }

    /**
     * Update turret alignment
     * @return false if limelight not connected (should exit OpMode)
     */
    public boolean updateTurret() {
        return turret.update();
    }

    public double getDistance() {
        return turret.getDistance();
    }

    public void updateDriveTrain(double forward, double right, double rotate) {
        driveTrain.update(forward, right, rotate);
    }

    public void updateSpindexer(boolean gamepadA, boolean gamepadB, boolean gamepadX, 
                                boolean gamepadY) {
        spindexer.update(gamepadA, gamepadB, gamepadX, gamepadY);
    }

    public void updateIntake() {
        intake.update();
    }

    // ==================== LAUNCHER DIRECT ACCESS ====================
    
    /**
     * Start the flywheel spinning
     */
    public void startFlywheel() {
        launcher.setSpinning(true);
        launcher.update();
    }

    /**
     * Start the flywheel with specified power
     * @param power Power level (0.0 to 1.0)
     */
    public void startFlywheel(double power) {
        launcher.setPower(power);
        launcher.setSpinning(true);
    }

    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        launcher.setSpinning(false);
        launcher.update();
    }

    /**
     * Check if flywheel is currently spinning
     * @return true if flywheel is spinning
     */
    public boolean isFlywheelSpinning() {
        return launcher.isSpinning();
    }

    /**
     * Set flywheel power (does not start/stop, just sets power level)
     * @param power Power level (0.0 to 1.0)
     */
    public void setFlywheelPower(double power) {
        launcher.setPower(power);
    }

    /**
     * Get current flywheel power
     * @return Current flywheel power (0.0 to 1.0)
     */
    public double getFlywheelPower() {
        return launcher.getPower();
    }

    /**
     * Set hood position
     * @param position Hood position (0.0 to 1.0)
     */
    public void setHoodPosition(double position) {
        launcher.setHoodPosition(position);
    }

    /**
     * Get current hood position
     * @return Current hood position (0.0 to 1.0)
     */
    public double getHoodPosition() {
        return launcher.getHoodPosition();
    }

    /**
     * Adjust hood position by increment
     * @param increment Amount to adjust (positive = up, negative = down)
     */
    public void adjustHood(double increment) {
        launcher.adjustHoodPosition(increment);
    }

    // ==================== INTAKE DIRECT ACCESS ====================

    /**
     * Start intake at default power
     */
    public void startIntake() {
        intake.start();
        intake.update();
    }

    /**
     * Start intake at specified power
     * @param power Intake power (-1.0 to 1.0, positive = forward)
     */
    public void startIntake(double power) {
        intake.start(power);
        intake.update();
    }

    /**
     * Stop intake
     */
    public void stopIntake() {
        intake.stop();
        intake.update();
    }

    /**
     * Reverse intake (eject)
     */
    public void reverseIntake() {
        intake.reverse();
        intake.update();
    }

    /**
     * Check if intake is running
     * @return true if intake is running
     */
    public boolean isIntakeRunning() {
        return intake.isRunning();
    }

    /**
     * Get current intake power
     * @return Current intake power (-1.0 to 1.0)
     */
    public double getIntakePower() {
        return intake.getPower();
    }

    /**
     * Get intake target power
     * @return Target power level (-1.0 to 1.0)
     */
    public double getIntakeTargetPower() {
        return intake.getTargetPower();
    }

    /**
     * Get intake state
     * @return Current intake state enum
     */
    public Intake.State getIntakeState() {
        return intake.getState();
    }

    /**
     * Check if intake ramping is enabled
     * @return true if ramping is enabled
     */
    public boolean isIntakeRampingEnabled() {
        return intake.isRampingEnabled();
    }

    /**
     * Get intake ramp rate
     * @return Ramp rate (power change per update)
     */
    public double getIntakeRampRate() {
        return intake.getRampRate();
    }

    // ==================== SPINDEXER DIRECT ACCESS ====================

    /**
     * Check if spindexer is ready to shoot (all balls intaked)
     * @return true if ready to shoot
     */
    public boolean isReadyToShoot() {
        return spindexer.shouldStopFlywheel() == false; // Simplified check
    }

    /**
     * Check if previous X button state (for edge detection)
     * @return true if X was pressed in previous update
     */
    public boolean wasShootButtonPressed() {
        return spindexer.isPrevX();
    }

    /**
     * Manually trigger a shot
     */
    public void shoot() {
        spindexer.shootBall();
    }

    /**
     * Rotate spindexer one division
     */
    public void rotateSpindexer() {
        spindexer.rotateOneDivision();
    }

    /**
     * Kick spoon (faster kicker sequence)
     */
    public void kickSpoon() {
        spindexer.kick();
    }

    /**
     * Get spindexer ball count
     * @return Number of balls detected by spindexer (0-3)
     */
    public int getSpindexerBallCount() {
        return spindexer.getBallCount();
    }

    /**
     * Check if all balls are intaked
     * @return true if all 3 balls are intaked
     */
    public boolean areAllBallsIntaked() {
        return spindexer.areAllBallsIntaked();
    }

    /**
     * Check if color sensing is currently active
     * @return true if color sensing is active
     */
    public boolean isColorSensingActive() {
        return spindexer.isSensing();
    }

    /**
     * Get current hue value from color sensor
     * @return Current hue value, or -1 if not available
     */
    public float getCurrentHue() {
        return spindexer.getCurrentHue();
    }


    /**
     * Shoot three balls in sequence: purple, purple, green
     * Starts flywheel, waits for spin-up, shoots all three balls, then stops flywheel
     */
    public void shoot_three_balls() {
        // Start flywheel at full power
        startFlywheel(1.0);
        
        // Wait for flywheel to spin up to full speed (typically 1-2 seconds)
        // This ensures the first ball shoots at the correct velocity
        try {
            Thread.sleep(1500); // 1.5 second spin-up time
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        // Shoot all three balls in sequence
        // Each call will: spindex to needed ball, shoot, advance flag to next color
        spindexer.shootBall(); // First ball (purple)
        spindexer.shootBall(); // Second ball (purple)
        spindexer.shootBall(); // Third ball (green)
        
        // Stop flywheel after all shots are complete
        stopFlywheel();
    }

    // ==================== TURRET DIRECT ACCESS ====================

    /**
     * Check if turret is aligned with target
     * @return true if aligned (limelight connected and on target)
     */
    public boolean isTurretAligned() {
        // Turret update returns true if limelight connected
        // We'd need to add a method to Turret to check alignment state
        return turret != null; // Placeholder - would need Turret to expose alignment state
    }

    // ==================== LIFT/PARKING DIRECT ACCESS ====================

    /**
     * Move lift up (parking position)
     */
    public void parkUp() {
        lift.moveUp();
    }

    /**
     * Move lift down (unparked position)
     */
    public void parkDown() {
        lift.moveDown();
    }

    /**
     * Update parking with gamepad inputs
     * @param dpadUp D-Pad up button state
     * @param dpadDown D-Pad down button state
     */
    public void updateParking(boolean dpadUp, boolean dpadDown) {
        lift.update(dpadUp, dpadDown);
    }

    /**
     * Get current lift state
     * @return Lift state (IDLE, MOVING_UP, MOVING_DOWN, HOLDING)
     */
    public Lift.State getLiftState() {
        return lift.getState();
    }

    /**
     * Check if lift is at top position
     * @return true if lift is at or near top position
     */
    public boolean isLiftAtTop() {
        Lift.State state = lift.getState();
        return state == Lift.State.HOLDING && lift.getLeftPosition() > 10000; // Approximate top check
    }

    /**
     * Check if lift is at bottom position
     * @return true if lift is at or near bottom position
     */
    public boolean isLiftAtBottom() {
        Lift.State state = lift.getState();
        return state == Lift.State.HOLDING && lift.getLeftPosition() < 100; // Approximate bottom check
    }

    // ==================== DRIVETRAIN DIRECT ACCESS ====================

    /**
     * Stop all drive motors
     */
    public void stopDriving() {
        driveTrain.stopMotors();
    }

    // ==================== HIGH-LEVEL WORKFLOW METHODS ====================

    /**
     * Prepare robot for shooting sequence
     * Starts flywheel and ensures turret is aligned
     * @return true if ready to shoot
     */
    public boolean prepareToShoot() {
        startFlywheel();
        return updateTurret(); // Returns false if limelight not connected
    }

    /**
     * Complete shooting sequence: shoot and check if should stop flywheel
     * @return true if should stop flywheel (3 shots completed)
     */
    public boolean shootSequence() {
        spindexer.shootBall();
        return spindexer.shouldStopFlywheel();
    }

    /**
     * Stop all robot movement and subsystems
     */
    public void stopAll() {
        stopFlywheel();
        stopIntake();
        stopDriving();
    }

    // Component getters (for advanced access if needed)
    public Turret getTurret() {
        return turret;
    }

    public Launcher getLauncher() {
        return launcher;
    }

    public Spindexer getSpindexer() {
        return spindexer;
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Lift getLift() {
        return lift;
    }

    public Intake getIntake() {
        return intake;
    }
    
    // ==================== COLOR SENSING AND BALL TRACKING ====================
    
    /**
     * Callback method called when a ball is detected by the color sensor
     * @param color Detected color ("purple" or "green")
     */
    private void onBallDetected(String color) {
        if (ballCount < 3) {
            ballOrder[ballCount] = color;
            ballCount++;
<<<<<<< Updated upstream
            // Capture the hue value when ball is detected
            lastHueDetected = spindexer.getCurrentHue();
            // Assign detected ball to LAST_SLOT (slot 2)
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
=======
        }
        // Rotate spindexer by one cycle when ball is detected
        spindexer.rotateOneDivision();
>>>>>>> Stashed changes
    }
    
    /**
     * Start color sensing - begins continuous color detection
     * When a ball is detected, callback is invoked which handles rotation
     */
    public void startColorSensing() {
        spindexer.startSensing(this::onBallDetected);
    }
    
    /**
     * Stop color sensing
     */
    public void stopColorSensing() {
        spindexer.stopSensing();
    }
    
    /**
     * Update color sensing - should be called every loop iteration
     * This handles continuous color detection when sensing is active
     */
    public void updateSpindexerSensing() {
        spindexer.updateSensing();
    }
    
    /**
<<<<<<< Updated upstream
     * Reset ball tracking - clears the ball order array and slot tracking
=======
     * Reset ball tracking - clears the ball order array
>>>>>>> Stashed changes
     */
    public void resetBallTracking() {
        ballOrder[0] = null;
        ballOrder[1] = null;
        ballOrder[2] = null;
        ballCount = 0;
<<<<<<< Updated upstream
        
        // Initialize ball slot tracking
        ballSlots.put(INTAKE_SLOT, "none");
        ballSlots.put(LAST_SLOT, "none");
        ballSlots.put(LAUNCH_SLOT, "none");
        launchIndex = 0;
        lastHueDetected = -1f;
=======
>>>>>>> Stashed changes
    }
    
    /**
     * Get the current ball order array
     * @return Array of ball colors (may contain nulls if less than 3 balls)
     */
    public String[] getBallOrder() {
        return ballOrder.clone();
    }
    
    /**
     * Get the current number of balls tracked
     * @return Number of balls (0-3)
     */
    public int getBallCount() {
        return ballCount;
    }
    
    /**
     * Get the ball sequence as a string (e.g., "PPG", "PGP", "GPP")
     * @return String representation of ball sequence, or empty string if no balls
     */
    public String getBallSequence() {
        if (ballCount == 0) {
            return "";
        }
        StringBuilder sequence = new StringBuilder();
        for (int i = 0; i < ballCount; i++) {
            if (ballOrder[i] != null) {
                // Use first letter: P for purple, G for green
                sequence.append(ballOrder[i].substring(0, 1).toUpperCase());
            }
        }
        return sequence.toString();
    }
<<<<<<< Updated upstream
    
    // ==================== BALL SLOT TRACKING METHODS ====================
    
    /**
     * Shifts balls between slots when spindexer rotates one division.
     * Rotation pattern (clockwise): INTAKE_SLOT -> LAST_SLOT -> LAUNCH_SLOT -> INTAKE_SLOT
     */
    public void shiftBallsBetweenSlots() {
        // Save current state
        String launchSlotBall = ballSlots.get(LAUNCH_SLOT);
        String lastSlotBall = ballSlots.get(LAST_SLOT);
        String intakeSlotBall = ballSlots.get(INTAKE_SLOT);

        // Rotate clockwise: LAUNCH -> INTAKE, INTAKE -> MIDDLE, MIDDLE -> LAUNCH
        ballSlots.put(INTAKE_SLOT, lastSlotBall != null ? lastSlotBall : "none");
        ballSlots.put(LAUNCH_SLOT, intakeSlotBall != null ? intakeSlotBall : "none");
        ballSlots.put(LAST_SLOT, launchSlotBall != null ? launchSlotBall : "none");
    }
    
    /**
     * Counts how many balls are currently in the slots (not "none").
     * @return Number of balls currently loaded (0-3)
     */
    public int getCurrentBallCount() {
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
    public int findSlotWithColor(String color) {
        if (color == null) {
            return -1;
        }
        for (int slot : new int[]{LAUNCH_SLOT, INTAKE_SLOT, LAST_SLOT}) {
            String slotColor = ballSlots.get(slot);
            if (slotColor != null && slotColor.equalsIgnoreCase(color)) {
                return slot;
            }
        }
        return -1;
    }
    
    /**
     * Rotates the spindexer to move a ball from the specified slot to LAUNCH_SLOT.
     * Rotation pattern: INTAKE_SLOT -> LAUNCH_SLOT -> LAST_SLOT -> INTAKE_SLOT (clockwise)
     * 
     * NOTE: This method performs blocking rotations. Each rotation blocks for ~150ms,
     * so calling this method will freeze the OpMode loop for up to 300ms (for 2 rotations).
     * The robot will be unresponsive to gamepad inputs during this time.
     * 
     * @param sourceSlot The slot containing the ball to move (INTAKE_SLOT, LAST_SLOT, or LAUNCH_SLOT)
     */
    public void rotateBallToLaunchSlot(int sourceSlot) {
        if (sourceSlot == LAUNCH_SLOT) {
            // Ball is already in launch slot, no rotation needed
            return;
        }
        
        int rotationsNeeded;
        if (sourceSlot == INTAKE_SLOT) {
            // From INTAKE_SLOT to LAUNCH_SLOT: need 1 rotation
            // INTAKE -> LAUNCH (clockwise rotation)
            rotationsNeeded = 1;
        } else if (sourceSlot == LAST_SLOT) {
            // From LAST_SLOT to LAUNCH_SLOT: need 2 rotations
            // LAST -> INTAKE -> LAUNCH (clockwise rotation)
            rotationsNeeded = 2;
        } else {
            // Unknown slot, no rotation
            return;
        }
        
        // Perform the rotations (without blocking sleep - rotations are fast enough)
        for (int i = 0; i < rotationsNeeded; i++) {
            spindexer.rotateOneDivision();
            shiftBallsBetweenSlots();
        }
    }
    
    /**
     * Pseudo launch method that launches one ball at a time in the desired launch order.
     * Each call launches the next ball in sequence.
     * Rotates the spindexer to ensure the correct color ball is in the launch slot, then clears it.
     * @param telemetry Telemetry instance for displaying status messages
     */
    public void launchOne(Telemetry telemetry) {
        // Check if obelisk sequence has been detected
        if (detectedBallSequence == null || detectedBallSequence.isEmpty()) {
            telemetry.addLine("Obelisk sequence not detected yet! Cannot launch.");
            return;
        }
        
        // Check if we've already launched all balls
        if (launchIndex >= detectedBallSequence.size()) {
            telemetry.addLine("All balls launched! Load 3 more balls using Intake to launch again.");
            return;
        }
        
        // Guard clause: check if no balls loaded
        if (getCurrentBallCount() == 0) {
            telemetry.addLine("No balls loaded! Cannot launch.");
            return;
        }
        
        // Get the desired color for this launch
        String desiredColor = detectedBallSequence.get(launchIndex);
        telemetry.addLine("Launching ball " + (launchIndex+1) + "/3: " + desiredColor);
        
        // Find which slot contains the desired color ball
        int ballSlot = findSlotWithColor(desiredColor);
        
        if (ballSlot == -1) {
            telemetry.addLine("WARNING: " + desiredColor + " ball not found! Skipping...");
            launchIndex++; // Still increment to avoid getting stuck
            return;
        }
        
        // Rotate spindexer to move the ball to LAUNCH_SLOT
        rotateBallToLaunchSlot(ballSlot);
        
        // Simulate launch by clearing the launch slot
        ballSlots.put(LAUNCH_SLOT, "none");
        telemetry.addLine("Launched " + desiredColor + " ball!");
        
        // Move to next ball in launch order
        launchIndex++;
    }
    
    /**
     * Reset launch index to 0 (called when 3 balls loaded)
     */
    public void resetLaunchIndex() {
        launchIndex = 0;
    }
    
    /**
     * Get color in specified slot
     * @param slot Slot number (INTAKE_SLOT, LAUNCH_SLOT, or LAST_SLOT)
     * @return Color string, or "none" if empty, or null if invalid slot
     */
    public String getSlotColor(int slot) {
        String color = ballSlots.get(slot);
        return color != null ? color : "none";
    }
    
    /**
     * Clear launch slot after kick
     */
    public void clearLaunchSlot() {
        ballSlots.put(LAUNCH_SLOT, "none");
    }
    
    /**
     * Get current launch index
     * @return Current launch index (0-2)
     */
    public int getLaunchIndex() {
        return launchIndex;
    }
    
    /**
     * Get last detected hue value
     * @return Last detected hue, or -1 if none detected
     */
    public float getLastHueDetected() {
        return lastHueDetected;
    }
    
    public void resetSpindexer() {
        spindexer.reset();
        resetBallTracking();
    }
    /**
     * Add comprehensive intake, spindexer, and slot position telemetry
     * @param telemetry Telemetry instance to add data to
     */
    public void addIntakeTelemetry(Telemetry telemetry) {
        // Add intake-specific telemetry
        telemetry.addLine("");
        telemetry.addLine("=== INTAKE STATUS ===");
        telemetry.addData("Intake Power", "%.2f", getIntakePower());
        telemetry.addData("Intake Target", "%.2f", getIntakeTargetPower());
        telemetry.addData("Intake Status", isIntakeRunning() ? "Running" : "Stopped");
        telemetry.addData("Intake State", getIntakeState().toString());
        if (isIntakeRampingEnabled()) {
            telemetry.addData("Intake Ramping", "Yes (%.3f/update)", getIntakeRampRate());
        }
        
        telemetry.addLine("");
        telemetry.addLine("=== ADDITIONAL SPINDEXER INFO ===");
        telemetry.addData("Ball Count", getSpindexerBallCount());
        telemetry.addData("All Balls Intaked", areAllBallsIntaked() ? "Yes" : "No");
        telemetry.addData("Color Sensing Active", isColorSensingActive() ? "Yes" : "No");
        float currentHue = getCurrentHue();
        if (currentHue >= 0) {
            telemetry.addData("Color Sensor Hue", "%.2f", currentHue);
        } else {
            telemetry.addData("Color Sensor Hue", "N/A");
        }

        telemetry.addLine("");
        telemetry.addLine("=== SLOT POSITIONS ===");
        telemetry.addData("Balls Detected", getBallCount());
        telemetry.addData("Current Balls Loaded", getCurrentBallCount());
        if (detectedBallSequence != null && !detectedBallSequence.isEmpty()) {
            telemetry.addData("Launch Index", launchIndex + "/" + detectedBallSequence.size());
        } else {
            telemetry.addData("Launch Index", launchIndex + "/? (sequence not detected)");
        }
        String intakeSlotColor = getSlotColor(INTAKE_SLOT);
        String intakeDisplay = intakeSlotColor.equals("none") ? "EMPTY" : "●" + intakeSlotColor.toUpperCase();
        telemetry.addData("Slot 0 (INTAKE)", intakeDisplay);
        
        String launchSlotColor = getSlotColor(LAUNCH_SLOT);
        String launchDisplay = launchSlotColor.equals("none") ? "EMPTY" : "●" + launchSlotColor.toUpperCase();
        telemetry.addData("Slot 1 (LAUNCH)", launchDisplay);
        
        String lastSlotColor = getSlotColor(LAST_SLOT);
        String middleDisplay = lastSlotColor.equals("none") ? "EMPTY" : "●" + lastSlotColor.toUpperCase();
        telemetry.addData("Slot 2 (MIDDLE)", middleDisplay);
        
        // Display hue of last detected ball
        if (lastHueDetected >= 0) {
            telemetry.addData("Last Ball Hue", "%.2f", lastHueDetected);
        } else {
            telemetry.addData("Last Ball Hue", "N/A");
        }
    }
    
    // ==================== OBELISK DETECTION METHODS ====================
    
    /**
     * Maps AprilTag ID to corresponding ball sequence
     * @param tagId The AprilTag ID detected (21, 22, or 23)
     * @return List of ball colors in sequence, or null if tag ID is not recognized
     */
    private List<String> mapTagIdToSequence(int tagId) {
        switch (tagId) {
            case APRILTAG_ID_PPG:
                // Purple - Purple - Green
                return Arrays.asList("purple", "purple", "green");
            
            case APRILTAG_ID_PGP:
                // Purple - Green - Purple
                return Arrays.asList("purple", "green", "purple");
            
            case APRILTAG_ID_GPP:
                // Green - Purple - Purple
                return Arrays.asList("green", "purple", "purple");
            
            default:
                // Unknown tag ID
                return null;
        }
    }
    
    /**
     * Detects obelisk AprilTag and updates detectedBallSequence
     * Only updates when a new/different AprilTag is detected
     */
    public void detectObeliskSequence() {
        if (turret == null) {
            return;
        }
        
        // Detect obelisk AprilTag using turret's limelight
        int tagId = turret.detectObeliskAprilTag();
        
        // Only process if this is a new detection (different from last detected)
        if (tagId != -1 && tagId != lastDetectedObeliskTagId) {
            lastDetectedObeliskTagId = tagId;
            
            // Map AprilTag ID to ball sequence
            List<String> sequence = mapTagIdToSequence(tagId);
            
            if (sequence != null && !sequence.isEmpty()) {
                detectedBallSequence = new ArrayList<>(sequence);
            }
        } else if (tagId == -1 && detectedBallSequence == null) {
            // No tag detected and we haven't detected sequence yet - reset last detected tag
            // This allows re-detection if we temporarily lose sight of the tag
            lastDetectedObeliskTagId = -1;
        }
    }
    
    /**
     * Get the detected ball sequence from obelisk
     * @return List of ball colors in sequence, or null if not yet detected
     */
    public List<String> getDetectedBallSequence() {
        return detectedBallSequence;
    }
    
    /**
     * Check if obelisk detection is still needed
     * @return true if detectedBallSequence is null (detection needed), false if already detected
     */
    public boolean needsObeliskDetection() {
        return detectedBallSequence == null;
    }
=======
>>>>>>> Stashed changes
}

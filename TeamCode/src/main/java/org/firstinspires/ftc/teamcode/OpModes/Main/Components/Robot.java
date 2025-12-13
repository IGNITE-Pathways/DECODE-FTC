package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    /**
     * Initialize all robot components
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     * @param opMode LinearOpMode instance (needed for Spindexer)
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        // Initialize all components
        turret = new Turret();
        turret.initialize(hardwareMap, telemetry);

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

    public void updateDriveTrain(double forward, double right, double rotate, boolean crossPressed) {
        driveTrain.update(forward, right, rotate, crossPressed);
    }

    public void updateSpindexer(boolean gamepadA, boolean gamepadB, boolean gamepadX, 
                                boolean gamepadY, boolean gamepadLeftBumper) {
        spindexer.update(gamepadA, gamepadB, gamepadX, gamepadY, gamepadLeftBumper);
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
        spindexer.shoot_ball_new(); // First ball (purple)
        spindexer.shoot_ball_new(); // Second ball (purple)
        spindexer.shoot_ball_new(); // Third ball (green)
        
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
        }
        // Rotate spindexer by one cycle when ball is detected
        spindexer.rotateOneDivision();
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
     * Reset ball tracking - clears the ball order array
     */
    public void resetBallTracking() {
        ballOrder[0] = null;
        ballOrder[1] = null;
        ballOrder[2] = null;
        ballCount = 0;
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
}

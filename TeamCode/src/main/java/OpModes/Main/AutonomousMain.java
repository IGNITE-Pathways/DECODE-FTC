package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;
import org.firstinspires.ftc.teamcode.OpModes.pedroPathing.Constants;

/**
 * Main autonomous routine using Pedro Pathing
 * Handles shooting pre-loaded balls, picking up balls, and shooting sequences
 */
public class AutonomousMain {
    public enum Alliance {
        RED, BLUE
    }
    
    public enum StartingPosition {
        LEFT, RIGHT
    }
    
    private final Alliance alliance;
    private final StartingPosition startingPosition;
    private final LinearOpMode opMode;
    private Follower follower;
    private Robot robot;
    
    // Field coordinates (placeholders - to be tuned)
    private Pose startingPose;
    private Pose shootingPose;
    private Pose pickupLocation1;
    private Pose pickupLocation2;
    
    public AutonomousMain(Alliance alliance, StartingPosition startingPosition, LinearOpMode opMode) {
        this.alliance = alliance;
        this.startingPosition = startingPosition;
        this.opMode = opMode;
    }
    
    /**
     * Initialize follower and robot components
     */
    public void initialize() {
        // Create Pedro Pathing follower
        follower = Constants.createFollower(opMode.hardwareMap);
        
        // Initialize robot components
        robot = new Robot();
        robot.initialize(opMode.hardwareMap, opMode.telemetry, opMode);
        
        // Calculate field positions based on alliance and starting position
        calculateFieldPositions();
        
        // Set starting pose
        follower.setStartingPose(startingPose);
        
        // Activate all PIDFs for path following
        follower.activateAllPIDFs();
        
        opMode.telemetry.addData("Alliance", alliance);
        opMode.telemetry.addData("Starting Position", startingPosition);
        opMode.telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, Heading: %.1f", 
            startingPose.getX(), startingPose.getY(), Math.toDegrees(startingPose.getHeading()));
        opMode.telemetry.update();
    }
    
    /**
     * Calculate field positions based on alliance and starting position
     * These are placeholder coordinates that need to be tuned
     */
    private void calculateFieldPositions() {
        double x, y;
        double heading = 0; // Default heading (facing forward)
        
        // Calculate starting position
        if (alliance == Alliance.RED) {
            if (startingPosition == StartingPosition.LEFT) {
                x = 72;
                y = 72;
                heading = Math.toRadians(0); // Facing forward
            } else { // RIGHT
                x = 72;
                y = -72;
                heading = Math.toRadians(0);
            }
        } else { // BLUE
            if (startingPosition == StartingPosition.LEFT) {
                x = -72;
                y = 72;
                heading = Math.toRadians(180); // Facing backward (opposite side)
            } else { // RIGHT
                x = -72;
                y = -72;
                heading = Math.toRadians(180);
            }
        }
        
        startingPose = new Pose(x, y, heading);
        
        // Shooting position (near center, facing target)
        if (alliance == Alliance.RED) {
            shootingPose = new Pose(0, 0, Math.toRadians(0));
        } else {
            shootingPose = new Pose(0, 0, Math.toRadians(180));
        }
        
        // Pickup locations (relative to starting side)
        if (alliance == Alliance.RED) {
            if (startingPosition == StartingPosition.LEFT) {
                pickupLocation1 = new Pose(36, 36, Math.toRadians(0));
                pickupLocation2 = new Pose(36, -36, Math.toRadians(0));
            } else {
                pickupLocation1 = new Pose(36, -36, Math.toRadians(0));
                pickupLocation2 = new Pose(36, 36, Math.toRadians(0));
            }
        } else { // BLUE
            if (startingPosition == StartingPosition.LEFT) {
                pickupLocation1 = new Pose(-36, 36, Math.toRadians(180));
                pickupLocation2 = new Pose(-36, -36, Math.toRadians(180));
            } else {
                pickupLocation1 = new Pose(-36, -36, Math.toRadians(180));
                pickupLocation2 = new Pose(-36, 36, Math.toRadians(180));
            }
        }
    }
    
    /**
     * Main autonomous sequence
     */
    public void run() {
        opMode.telemetry.addLine("Starting Autonomous Sequence");
        opMode.telemetry.update();
        
        // Step 1: Shoot 3 pre-loaded balls
        opMode.telemetry.addLine("Step 1: Shooting 3 pre-loaded balls");
        opMode.telemetry.update();
        shootThreeBalls();
        
        // Step 2: Path to ball pickup location #1
        opMode.telemetry.addLine("Step 2: Moving to pickup location #1");
        opMode.telemetry.update();
        pathToLocation(pickupLocation1);
        
        // Step 3: Intake 3 balls
        opMode.telemetry.addLine("Step 3: Intaking 3 balls");
        opMode.telemetry.update();
        intakeThreeBalls();
        
        // Step 4: Path to shooting position
        opMode.telemetry.addLine("Step 4: Moving to shooting position");
        opMode.telemetry.update();
        pathToLocation(shootingPose);
        
        // Step 5: Shoot 3 balls
        opMode.telemetry.addLine("Step 5: Shooting 3 balls");
        opMode.telemetry.update();
        shootThreeBalls();
        
        // Step 6: Path to ball pickup location #2
        opMode.telemetry.addLine("Step 6: Moving to pickup location #2");
        opMode.telemetry.update();
        pathToLocation(pickupLocation2);
        
        // Step 7: Intake 3 balls
        opMode.telemetry.addLine("Step 7: Intaking 3 balls");
        opMode.telemetry.update();
        intakeThreeBalls();
        
        // Step 8: Path to shooting position
        opMode.telemetry.addLine("Step 8: Moving to shooting position");
        opMode.telemetry.update();
        pathToLocation(shootingPose);
        
        // Step 9: Shoot 3 balls
        opMode.telemetry.addLine("Step 9: Shooting 3 balls");
        opMode.telemetry.update();
        shootThreeBalls();
        
        // Stop all systems
        robot.stopAll();
        opMode.telemetry.addLine("Autonomous Complete!");
        opMode.telemetry.update();
    }
    
    /**
     * Shoot 3 balls sequence
     */
    private void shootThreeBalls() {
        // Start flywheel
        robot.startFlywheel();
        opMode.sleep(1500); // Give flywheel time to spin up
        
        // Align turret - keep updating until aligned or timeout
        int alignmentAttempts = 0;
        int maxAlignmentAttempts = 150; // 15 seconds at 100ms sleep
        boolean aligned = false;
        
        while (opMode.opModeIsActive() && alignmentAttempts < maxAlignmentAttempts && !aligned) {
            robot.updateTurret();
            robot.updateLauncher();
            
            opMode.telemetry.addData("Aligning Turret", "Attempt " + alignmentAttempts);
            opMode.telemetry.update();
            
            opMode.sleep(100);
            alignmentAttempts++;
            
            // Give turret time to align - assume aligned after reasonable wait
            // The turret's update() method handles alignment internally
            if (alignmentAttempts > 30) {
                aligned = true; // Proceed with shooting even if not perfectly aligned
            }
        }
        
        opMode.telemetry.addLine("Turret aligned. Starting to shoot...");
        opMode.telemetry.update();
        
        // Shoot 3 balls using shootSequence
        // shootSequence() handles one shot and returns true if should stop flywheel
        for (int i = 0; i < 3 && opMode.opModeIsActive(); i++) {
            boolean shouldStop = robot.shootSequence();
            
            opMode.telemetry.addData("Shot", (i + 1) + " / 3");
            opMode.telemetry.update();
            
            // Small delay between shots to allow kicker to reset
            opMode.sleep(800);
            
            if (shouldStop) {
                break; // All 3 shots completed
            }
        }
        
        // Stop flywheel after shooting
        robot.stopFlywheel();
        opMode.sleep(500); // Brief pause after shooting
    }
    
    /**
     * Intake 3 balls sequence
     */
    private void intakeThreeBalls() {
        // Start intake
        robot.startIntake();
        
        // Wait for spindexer to detect 3 balls
        int maxWaitTime = 15000; // 15 seconds max
        int waitTime = 0;
        int ballCount = 0;
        
        while (opMode.opModeIsActive() && waitTime < maxWaitTime && ballCount < 3) {
            // Update spindexer to detect balls (color detection happens in update)
            robot.updateSpindexer(false, false, false, false, false);
            
            // Check ball count from spindexer
            ballCount = robot.getSpindexer().getBallCount();
            
            opMode.telemetry.addData("Balls Intaked", ballCount + " / 3");
            opMode.telemetry.addData("All Balls Intaked", robot.getSpindexer().areAllBallsIntaked());
            opMode.telemetry.update();
            
            opMode.sleep(100);
            waitTime += 100;
        }
        
        // Stop intake
        robot.stopIntake();
        
        opMode.telemetry.addLine("Intake complete. Balls: " + ballCount);
        opMode.telemetry.update();
        
        // Small delay to ensure balls are settled
        opMode.sleep(500);
    }
    
    /**
     * Create and follow a path to the target location
     */
    private void pathToLocation(Pose target) {
        // Get current pose
        Pose currentPose = follower.getPose();
        
        // Create path using pathBuilder (returns PathChain, not Path)
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, target))
                .setConstantHeadingInterpolation(target.getHeading())
                .build();
        
        // Follow the path (followPath accepts both Path and PathChain)
        follower.followPath(path);
        
        // Update follower until path is complete
        while (opMode.opModeIsActive() && follower.isBusy()) {
            follower.update();
            robot.updateTurret(); // Keep turret updated during movement
            robot.updateLauncher(); // Keep launcher updated
            
            Pose current = follower.getPose();
            opMode.telemetry.addData("Following Path", "X: %.1f, Y: %.1f, Heading: %.1f", 
                current.getX(), current.getY(), Math.toDegrees(current.getHeading()));
            opMode.telemetry.addData("Target", "X: %.1f, Y: %.1f", target.getX(), target.getY());
            opMode.telemetry.update();
            opMode.sleep(10);
        }
        
        opMode.telemetry.addLine("Path complete");
        opMode.telemetry.update();
        
        // Small delay after path completion
        opMode.sleep(200);
    }
}

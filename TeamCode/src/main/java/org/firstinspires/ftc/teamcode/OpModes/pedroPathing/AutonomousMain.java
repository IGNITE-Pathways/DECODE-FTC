package org.firstinspires.ftc.teamcode.OpModes.pedroPathing;

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
                x = 56;
                y = 8;
                heading = Math.toRadians(90);
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
     * Follow a PathChain while updating robot systems
     */
    private void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);
        
        // Update follower until path is complete
        while (opMode.opModeIsActive() && follower.isBusy()) {
            follower.update();
            robot.updateTurret(); // Keep turret updated during movement
            robot.updateLauncher(); // Keep launcher updated
            
            Pose current = follower.getPose();
            opMode.telemetry.addData("Following Path", "X: %.1f, Y: %.1f, Heading: %.1f", 
                current.getX(), current.getY(), Math.toDegrees(current.getHeading()));
            opMode.telemetry.update();
            opMode.sleep(10);
        }
        
        opMode.telemetry.addLine("Path segment complete");
        opMode.telemetry.update();
        
        // Small delay after path completion
        opMode.sleep(200);
    }
    
    /**
     * Main autonomous sequence
     */
    public void run() {
        opMode.telemetry.addLine("Starting Autonomous Sequence");
        opMode.telemetry.update();
        
        // Check if this is RedRight - use custom path
        if (alliance == Alliance.RED && startingPosition == StartingPosition.RIGHT) {
            runRedRightPath();
        } else {
            // Use generic path for other alliance/position combinations
            runGenericPath();
        }
        
        // Stop all systems
        robot.stopAll();
        opMode.telemetry.addLine("Autonomous Complete!");
        opMode.telemetry.update();
    }
    
    /**
     * Run the RedRight specific path with actions at segment endpoints
     */
    private void runRedRightPath() {
        opMode.telemetry.addLine("Running RedRight Path");
        opMode.telemetry.update();
        
        // Build path chains for each section to allow actions between segments
        // Section 1: Segments 1-2 (align + first shooting)
        PathChain section1 = buildRedRightPathSection1();

        followPathChain(section1);
        
        // Action after segment 2: Shoot 3 pre-loaded balls
        opMode.telemetry.addLine("Shooting 3 pre-loaded balls");
        opMode.telemetry.update();
        shootThreeBalls();
        
        // Section 2: Segments 3-9 (align with set 1 + get balls)
        PathChain section2 = buildRedRightPathSection2();
        // Start intake before moving to balls
        robot.startIntake();
        followPathChain(section2);
        // Action after segment 9: Intake 3 balls
        opMode.telemetry.addLine("Intaking 3 balls");
        opMode.telemetry.update();
        intakeThreeBalls();
        
        // Section 3: Segments 10-12 (reset to shoot + shooting)
        PathChain section3 = buildRedRightPathSection3();
        followPathChain(section3);
        // Action after segment 12: Shoot 3 balls
        opMode.telemetry.addLine("Shooting 3 balls");
        opMode.telemetry.update();
        shootThreeBalls();
        
        // Section 4: Segments 13-19 (align with set 2 + get balls)
        PathChain section4 = buildRedRightPathSection4();
        // Start intake before moving to balls
        robot.startIntake();
        followPathChain(section4);
        // Action after segment 19: Intake 3 balls
        opMode.telemetry.addLine("Intaking 3 balls");
        opMode.telemetry.update();
        intakeThreeBalls();
        
        // Section 5: Segments 20-22 (reset to shoot + shooting)
        PathChain section5 = buildRedRightPathSection5();
        followPathChain(section5);
        // Action after segment 22: Shoot 3 balls
        opMode.telemetry.addLine("Shooting 3 balls");
        opMode.telemetry.update();
        shootThreeBalls();
        
        // Section 6: Segment 23 (final path)
        PathChain section6 = buildRedRightPathSection6();
        followPathChain(section6);
    }
    
    /**
     * Run generic path for non-RedRight autonomous
     */
    private void runGenericPath() {
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
    }
    
    /**
     * Build RedRight path section 1: Segments 1-2 (align + first shooting)
     */
    private PathChain buildRedRightPathSection1() {
        Pose currentPose = new Pose(56, 8, Math.toRadians(90));
        
        // Segment 1: "align" - (56,8) to (56,12), linear heading 90→115
        Pose seg1End = new Pose(56, 12, Math.toRadians(115));
        currentPose = seg1End;
        
        // Segment 2: "shooting" - (56,12) to (56,12), linear heading 115→115
        Pose seg2End = new Pose(56, 12, Math.toRadians(115));
        
        return follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), seg1End))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .addPath(new BezierLine(seg1End, seg2End))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(115))
                .setGlobalDeceleration()
                .build();
    }
    
    /**
     * Build RedRight path section 2: Segments 3-9 (align with set 1 + get balls)
     */
    private PathChain buildRedRightPathSection2() {
        Pose currentPose = follower.getPose();
        Pose seg3End = new Pose(40, 39, Math.toRadians(180));
        Pose seg4End = new Pose(35, 39, seg3End.getHeading());
        Pose seg5End = new Pose(35, 39, seg4End.getHeading());
        Pose seg6End = new Pose(29, 39, seg5End.getHeading());
        Pose seg7End = new Pose(29, 39, seg6End.getHeading());
        Pose seg8End = new Pose(22, 39, seg7End.getHeading());
        Pose seg9End = new Pose(22, 39, seg8End.getHeading());
        
        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, seg3End))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .addPath(new BezierLine(seg3End, seg4End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg4End, seg5End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg5End, seg6End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg6End, seg7End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg7End, seg8End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg8End, seg9End))
                .setTangentHeadingInterpolation()
                .build();
    }
    
    /**
     * Build RedRight path section 3: Segments 10-12 (reset to shoot + shooting)
     */
    private PathChain buildRedRightPathSection3() {
        Pose currentPose = follower.getPose();
        Pose seg10End = new Pose(56, 12, Math.toRadians(115));
        Pose seg11End = new Pose(56, 12, Math.toRadians(115));
        Pose seg12End = new Pose(56, 12, Math.toRadians(115));
        
        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, seg10End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .addPath(new BezierLine(seg10End, seg11End))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(115))
                .addPath(new BezierLine(seg11End, seg12End))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(115))
                .build();
    }
    
    /**
     * Build RedRight path section 4: Segments 13-19 (align with set 2 + get balls)
     */
    private PathChain buildRedRightPathSection4() {
        Pose currentPose = follower.getPose();
        Pose seg13End = new Pose(40, 65, Math.toRadians(180));
        Pose seg14End = new Pose(35, 65, seg13End.getHeading());
        Pose seg15End = new Pose(35, 65, seg14End.getHeading());
        Pose seg16End = new Pose(29, 65, seg15End.getHeading());
        Pose seg17End = new Pose(29, 65, seg16End.getHeading());
        Pose seg18End = new Pose(22, 65, seg17End.getHeading());
        Pose seg19End = new Pose(22, 65, seg18End.getHeading());
        
        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, seg13End))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .addPath(new BezierLine(seg13End, seg14End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg14End, seg15End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg15End, seg16End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg16End, seg17End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg17End, seg18End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg18End, seg19End))
                .setTangentHeadingInterpolation()
                .build();
    }
    
    /**
     * Build RedRight path section 5: Segments 20-22 (reset to shoot + shooting)
     */
    private PathChain buildRedRightPathSection5() {
        Pose currentPose = follower.getPose();
        Pose seg20End = new Pose(56, 110, Math.toRadians(150));
        Pose seg21End = new Pose(56, 110, seg20End.getHeading());
        Pose seg22End = new Pose(56, 110, seg21End.getHeading());
        
        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, seg20End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .addPath(new BezierLine(seg20End, seg21End))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(seg21End, seg22End))
                .setTangentHeadingInterpolation()
                .build();
    }
    
    /**
     * Build RedRight path section 6: Segment 23 (final path)
     */
    private PathChain buildRedRightPathSection6() {
        Pose currentPose = follower.getPose();
        Pose seg23End = new Pose(56, 50, Math.toRadians(150));
        
        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, seg23End))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                .build();
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

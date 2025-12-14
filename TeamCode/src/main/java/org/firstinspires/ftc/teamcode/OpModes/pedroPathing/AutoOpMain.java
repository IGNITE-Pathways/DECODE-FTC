package org.firstinspires.ftc.teamcode.OpModes.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

/**
 * Base Autonomous class - DO NOT RUN DIRECTLY
 * Use AutoOpMainBlue or AutoOpMainRed instead
 */
public class AutoOpMain extends OpMode {
    private Follower follower;
    private Robot robot;
    private Timer pathTimer, opmodeTimer, waitTimer;

    private int pathState;
    private boolean isWaiting = false;
    private double customWaitTime = 1.5; // Custom wait time for current wait state (default to WAIT_TIME_SECONDS)
    private static final double WAIT_TIME_SECONDS = 1.5; // Wait 1.5 seconds after each path
    private static final double BALL_WAIT_TIME_SECONDS = 2.0; // Longer wait for ball collection paths
    private static final double FLYWHEEL_SPINUP_TIME = 1.5; // Time for flywheel to spin up
    private static final double SPINDEXER_ROTATION_WAIT_TIME = 0.5; // Time to wait for spindexer rotation to complete
    private static final double KICKER_WAIT_TIME = 0.8; // Time to wait for kicker sequence
    
    // Ball preloading state tracking
    private boolean ballsPreloaded = false; // Track if balls have been preloaded

    // Alliance color: BLUE or RED
    protected AllianceColor allianceColor = null;
    
    /**
     * Set the alliance color for this op mode
     * @param color AllianceColor.BLUE or AllianceColor.RED
     */
    protected void setAllianceColor(AllianceColor color) {
        this.allianceColor = color;
    }

    // Starting position of the robot from trajectory (8).pp
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    // Path declarations - trajectory path segments (removing duplicates)
    private PathChain Path1; // align: (56,8) to (56,12) - BLUE alliance
    private PathChain Path1Red; // align: (88,8) to (88,12) - RED alliance (mirrored from Path1)
    private PathChain Path2; // align with set 1: (56,12) to (40,39)
    private PathChain Path3; // get ball 1: (40,39) to (35,39)
    private PathChain Path4; // get ball 2: (35,39) to (29,39)
    private PathChain Path5; // get ball 3: (29,39) to (22,39)
    private PathChain Path6; // reset to shoot: (22,39) to (56,12)
    private PathChain Path7; // align with set 2: (56,12) to (40,65)
    private PathChain Path8; // get ball 1: (40,65) to (35,65)
    private PathChain Path9; // get ball 2: (35,65) to (29,65)
    private PathChain Path10; // get ball 3: (29,65) to (22,65)
    private PathChain Path11; // reset to shoot: (22,65) to (56,110)
    private PathChain Path12; // final path: (56,110) to (56,50)

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot();
        // Note: Passing null for LinearOpMode since we're using OpMode, not LinearOpMode
        // Alliance color is passed for turret tag detection
        robot.initialize(hardwareMap, telemetry, null, allianceColor);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        // First call: set preloaded balls (assumed configuration)
        if (!ballsPreloaded) {
            robot.resetBallTracking();
            
            // Set preloaded balls according to assumed configuration:
            // LAST_SLOT (slot 2) = green
            // INTAKE_SLOT (slot 0) = purple
            // LAUNCH_SLOT (slot 1) = purple
            robot.setPreloadedBalls("purple", "purple", "green");
            
            // Also update Spindexer to match (divisions: 0=front/intake, 1=middle/launch, 2=back/last)
            robot.getSpindexer().setPreloadedBalls("purple", "purple", "green");
            
            ballsPreloaded = true;
        }
        
        // Display telemetry
        telemetry.addData("Status", "Initialized");
        if (allianceColor != null) {
            telemetry.addData("Alliance", allianceColor.name());
        }
        telemetry.addLine("");
        telemetry.addLine("=== BALL PRELOADING ===");
        telemetry.addData("Balls Preloaded", "Yes (Assumed Configuration)");
        telemetry.addData("Balls Loaded", robot.getCurrentBallCount() + "/3");
        telemetry.addData("Ball Sequence", robot.getBallSequence());
        
        telemetry.addLine("");
        telemetry.addLine("✓ READY TO PLAY - 3 balls preloaded!");
        telemetry.addLine("");
        telemetry.addLine("=== SLOT POSITIONS ===");
        
        // Display all three slots and their ball colors
        String intakeSlotColor = robot.getSlotColor(0); // INTAKE_SLOT
        String intakeDisplay = intakeSlotColor.equals("none") ? "EMPTY" : intakeSlotColor.toUpperCase();
        telemetry.addData("Slot 0 (INTAKE)", intakeDisplay);
        
        String launchSlotColor = robot.getSlotColor(1); // LAUNCH_SLOT
        String launchDisplay = launchSlotColor.equals("none") ? "EMPTY" : launchSlotColor.toUpperCase();
        telemetry.addData("Slot 1 (LAUNCH)", launchDisplay);
        
        String lastSlotColor = robot.getSlotColor(2); // LAST_SLOT
        String lastDisplay = lastSlotColor.equals("none") ? "EMPTY" : lastSlotColor.toUpperCase();
        telemetry.addData("Slot 2 (LAST)", lastDisplay);
        
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        
        // Read limelight Obelisk and populate detectedBallSequence in Robot
        robot.detectObeliskSequence();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        robot.updateIntake(); // Update intake to keep it running
        robot.updateSpindexerSensing(); // Update color sensing for ball detection
        
        // Keep trying to detect obelisk sequence until successful
        if (robot.needsObeliskDetection()) {
            robot.detectObeliskSequence();
        }
        
        // Update turret during shooting states (cases 15-19, 160, 170, 180) to maintain alignment
        if ((pathState >= 15 && pathState <= 19) || pathState == 160 || pathState == 170 || pathState == 180) {
            robot.updateTurret();
        }
        
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("is waiting", isWaiting);
        telemetry.addData("is busy", follower.isBusy());
        telemetry.addData("intake running", robot.isIntakeRunning());
        telemetry.addData("ball count", robot.getBallCount());
        telemetry.addData("ball sequence", robot.getBallSequence());
        
        // Display obelisk detection status
        if (robot.getDetectedBallSequence() != null) {
            telemetry.addData("Obelisk Sequence", robot.getDetectedBallSequence().toString());
        } else {
            telemetry.addData("Obelisk Sequence", "Not detected yet");
        }
        
        // Display shooting progress and status
        if ((pathState >= 15 && pathState <= 19) || pathState == 160 || pathState == 170 || pathState == 180) {
            telemetry.addLine("");
            telemetry.addLine("=== SHOOTING SEQUENCE ===");
            telemetry.addData("Flywheel Status", robot.isFlywheelSpinning() ? "Running" : "Stopped");
            telemetry.addData("Launch Index", robot.getLaunchIndex() + "/3");
            
            if (pathState == 15) {
                telemetry.addData("Status", "Preparing to shoot (flywheel spin-up)");
            } else if (pathState == 16 || pathState == 17 || pathState == 18) {
                int ballNumber = (pathState == 16) ? 1 : (pathState == 17) ? 2 : 3;
                telemetry.addData("Status", "Positioning ball " + ballNumber + "/3");
                if (robot.getDetectedBallSequence() != null && robot.getLaunchIndex() > 0) {
                    String currentBall = robot.getDetectedBallSequence().get(robot.getLaunchIndex() - 1);
                    telemetry.addData("Current Ball", currentBall);
                }
            } else if (pathState == 160 || pathState == 170 || pathState == 180) {
                int ballNumber = (pathState == 160) ? 1 : (pathState == 170) ? 2 : 3;
                telemetry.addData("Status", "Kicking ball " + ballNumber + "/3");
            } else if (pathState == 19) {
                telemetry.addData("Status", "Shooting complete!");
            }
        }
        
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        if (pathState >= 2 && pathState <= 4) {
            telemetry.addLine("Getting balls - Path " + (pathState - 1));
        }
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        // Handle waiting state
        if (isWaiting) {
            // Use custom wait time if set, otherwise use default logic
            double currentWaitTime = customWaitTime;
            
            if (waitTimer.getElapsedTimeSeconds() >= currentWaitTime) {
                isWaiting = false;
                waitTimer.resetTimer();
                customWaitTime = WAIT_TIME_SECONDS; // Reset to default
            } else {
                return; // Still waiting, don't process path states
            }
        }
        
        switch (pathState) {
            case 0:
                // Use Path1Red for RED alliance, Path1 for BLUE alliance
                if (allianceColor == AllianceColor.RED) {
                    follower.followPath(Path1Red);
                } else {
                    follower.followPath(Path1);
                }
                setPathState(1);
                break;
            case 1:
                // Path1 complete, start shooting sequence immediately
                if (!follower.isBusy()) {
                    robot.setFlywheelPower(0.84);
                    robot.startFlywheel();
                    // Start flywheel early
                    robot.setHoodPosition(0.75);
                    robot.resetLaunchIndex(); // Reset to shoot first ball
                    setPathState(15); // Transition to shooting prep
                }
                break;
            
            case 15: // Shooting preparation
                // Keep turret aligned during preparation
                robot.getTurret().setPositionDirect(0.6); //We should lock turret straight
                robot.setHoodPosition(0.75); //incase pop
                // Wait for flywheel spin-up
                if (pathTimer.getElapsedTimeSeconds() >= FLYWHEEL_SPINUP_TIME) {
                    setPathState(16); // Start shooting ball 1
                }
                break;
            
            case 16: // Position ball 1 for shooting
                if (!isWaiting) {
                    robot.getTurret().setPositionDirect(0.6);
                    robot.setHoodPosition(0.75);
                    robot.launchOne(telemetry); // Rotate spindexer to position ball 1
                    startWait(SPINDEXER_ROTATION_WAIT_TIME); // Wait for rotation to complete
                    setPathState(160); // Move to kick phase
                }
                break;
            
            case 160: // Kick ball 1
                if (!isWaiting) {
                    robot.kickSpoon(); // Fire kicker
                    startWait(KICKER_WAIT_TIME); // Wait for kicker sequence
                    setPathState(17); // Move to ball 2 positioning
                }
                break;
            
            case 17: // Position ball 2 for shooting
                if (!isWaiting) {
                    robot.getTurret().setPositionDirect(0.6);
                    robot.launchOne(telemetry); // Rotate spindexer to position ball 2
                    startWait(SPINDEXER_ROTATION_WAIT_TIME); // Wait for rotation to complete
                    setPathState(170); // Move to kick phase
                }
                break;
            
            case 170: // Kick ball 2
                if (!isWaiting) {
                    robot.kickSpoon(); // Fire kicker
                    startWait(KICKER_WAIT_TIME); // Wait for kicker sequence
                    setPathState(18); // Move to ball 3 positioning
                }
                break;
            
            case 18: // Position ball 3 for shooting
                if (!isWaiting) {
                    robot.getTurret().setPositionDirect(0.6);
                    robot.launchOne(telemetry); // Rotate spindexer to position ball 3
                    startWait(SPINDEXER_ROTATION_WAIT_TIME); // Wait for rotation to complete
                    setPathState(180); // Move to kick phase
                }
                break;
            
            case 180: // Kick ball 3
                if (!isWaiting) {
                    robot.kickSpoon(); // Fire kicker
                    startWait(KICKER_WAIT_TIME); // Wait for kicker sequence
                    setPathState(19); // Move to completion
                }
                break;
            
            case 19: // Shooting complete
                if (!isWaiting) {
                    robot.stopFlywheel();
                    setPathState(2); // Transition to Path2 after shooting
                }
                break;
            
            case 2: // Follow Path2 after shooting all 3 balls
                // Start Path2 on first entry (follower not busy yet)
                if (!follower.isBusy()) {
                    follower.followPath(Path2);
                    setPathState(3); // Move to waiting state
                }
                break;
            
            case 3: // Wait for Path2 to complete
                if (!follower.isBusy()) {
                    // Path2 complete, end autonomous routine
                    setPathState(-1);
                }
                break;
            /*
            case 3:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path3); // Get ball 1
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path4); // Get ball 2
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path5); // Get ball 3
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    robot.stopIntake(); // Stop intake after getting all balls from set 1
                    robot.stopColorSensing(); // Stop color sensing
                    startWait();
                    follower.followPath(Path6);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    robot.startIntake(); // Start intake before aligning with set 2
                    robot.startColorSensing(); // Start color sensing for ball detection
                    startWait();
                    follower.followPath(Path7);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path8);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path9);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path10);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    robot.stopIntake(); // Stop intake after getting all balls from set 2
                    robot.stopColorSensing(); // Stop color sensing
                    startWait();
                    follower.followPath(Path11);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path12);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    // Set the state to a Case we won't use or define, so it just stops running any new paths
                    setPathState(-1);
                }
                break;*/
        }
    }
    
    private void startWait() {
        isWaiting = true;
        waitTimer.resetTimer();
    }
    
    private void startWait(double waitTime) {
        isWaiting = true;
        customWaitTime = waitTime;
        waitTimer.resetTimer();
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        // Path 1: align - (56,8) to (56,12), linear heading 90→115 (BLUE alliance)
        Path1 = follower.pathBuilder()
                .setGlobalDeceleration(2.0) // Higher deceleration slows paths down significantly
                .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        // Path 1Red: align - (88,8) to (88,12), linear heading 90→65 (RED alliance, mirrored from Path1)
        // Mirroring left-to-right: x -> 144 - x, heading -> 180° - heading
        Path1Red = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(88, 8, Math.toRadians(90)), new Pose(88, 12, Math.toRadians(65))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                .build();

        // Path 2: align with set 1 - (56,12) to (40,39), linear heading 115→180
        Path2 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 39, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        // Path 3: get ball 1 - (40,39) to (35,39), tangential heading
        Path3 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(40, 39, Math.toRadians(180)), new Pose(35, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 4: get ball 2 - (35,39) to (29,39), tangential heading
        Path4 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(35, 39, Math.toRadians(180)), new Pose(29, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5: get ball 3 - (29,39) to (22,39), tangential heading
        Path5 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(29, 39, Math.toRadians(180)), new Pose(22, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 6: reset to shoot - (22,39) to (56,12), linear heading 180→115
        Path6 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 39, Math.toRadians(180)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        // Path 7: align with set 2 - (56,12) to (40,65), linear heading 115→180
        Path7 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 65, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        // Path 8: get ball 1 - (40,65) to (35,65), tangential heading
        Path8 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(40, 65, Math.toRadians(180)), new Pose(35, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 9: get ball 2 - (35,65) to (29,65), tangential heading
        Path9 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(35, 65, Math.toRadians(180)), new Pose(29, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 10: get ball 3 - (29,65) to (22,65), tangential heading
        Path10 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(29, 65, Math.toRadians(180)), new Pose(22, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 11: reset to shoot - (22,65) to (56,110), linear heading 180→150
        Path11 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 65, Math.toRadians(180)), new Pose(56, 110, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();

        // Path 12: final path - (56,110) to (56,50), linear heading 150→150
        Path12 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 110, Math.toRadians(150)), new Pose(56, 50, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                .build();
    }

    @Override
    public void stop() {}
}

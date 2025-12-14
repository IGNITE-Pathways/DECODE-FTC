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
    private static final double KICKER_WAIT_TIME = 0.8; // Time to wait for kicker sequence
    
    // Ball preloading state tracking
    private boolean intakeStarted = false; // Track if intake has been started
    private boolean ballsLoaded = false; // Track if 3 balls have been loaded
    private boolean path2Started = false; // Track if Path2 has been started
    
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
    private PathChain Path1; // align: (56,8) to (56,12)
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
        // First call: reset ball tracking and start intake
        if (!intakeStarted) {
            robot.resetBallTracking();
            robot.startIntake();
            robot.startColorSensing();
            intakeStarted = true;
        }
        
        // Update intake and color sensing every loop iteration
        robot.updateIntake();
        robot.updateSpindexerSensing();
        
        // Check if 3 balls have been loaded
        // Note: onBallDetected callback already resets launchIndex when ballCount >= 3
        if (!ballsLoaded && robot.getCurrentBallCount() >= 3) {
            robot.stopIntake();
            robot.stopColorSensing();
            ballsLoaded = true;
        }
        
        // Display telemetry
        telemetry.addData("Status", "Initialized");
        if (allianceColor != null) {
            telemetry.addData("Alliance", allianceColor.name());
        }
        telemetry.addLine("");
        telemetry.addLine("=== BALL PRELOADING ===");
        telemetry.addData("Intake Status", robot.isIntakeRunning() ? "Running" : "Stopped");
        telemetry.addData("Color Sensing", robot.isColorSensingActive() ? "Active" : "Inactive");
        telemetry.addData("Balls Loaded", robot.getCurrentBallCount() + "/3");
        telemetry.addData("Ball Sequence", robot.getBallSequence());
        
        if (ballsLoaded) {
            telemetry.addLine("");
            telemetry.addLine("✓ READY TO PLAY - 3 balls loaded!");
        } else {
            telemetry.addLine("");
            telemetry.addLine("Waiting for player to provide 3 balls...");
        }
        
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
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/pedroPathing/AutoOpMain.java
        
        // Keep trying to detect obelisk sequence until successful
        if (robot.needsObeliskDetection()) {
            robot.detectObeliskSequence();
        }
        
        // Update turret during shooting states (cases 15-19) to maintain alignment
        if (pathState >= 15 && pathState <= 19) {
            robot.updateTurret();
        }
        
=======
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/pedroPathing/AutonPedroDocumentation.java
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("is waiting", isWaiting);
        telemetry.addData("is busy", follower.isBusy());
        telemetry.addData("intake running", robot.isIntakeRunning());
        telemetry.addData("ball count", robot.getBallCount());
        telemetry.addData("ball sequence", robot.getBallSequence());
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/pedroPathing/AutoOpMain.java
        
        // Display obelisk detection status
        if (robot.getDetectedBallSequence() != null) {
            telemetry.addData("Obelisk Sequence", robot.getDetectedBallSequence().toString());
        } else {
            telemetry.addData("Obelisk Sequence", "Not detected yet");
        }
        
        // Display shooting progress and status
        if (pathState >= 15 && pathState <= 19) {
            telemetry.addLine("");
            telemetry.addLine("=== SHOOTING SEQUENCE ===");
            telemetry.addData("Flywheel Status", robot.isFlywheelSpinning() ? "Running" : "Stopped");
            telemetry.addData("Launch Index", robot.getLaunchIndex() + "/3");
            
            if (pathState == 15) {
                telemetry.addData("Status", "Preparing to shoot (flywheel spin-up)");
            } else if (pathState >= 16 && pathState <= 18) {
                int ballNumber = pathState - 15;
                telemetry.addData("Status", "Shooting ball " + ballNumber + "/3");
                if (robot.getDetectedBallSequence() != null && robot.getLaunchIndex() > 0) {
                    String currentBall = robot.getDetectedBallSequence().get(robot.getLaunchIndex() - 1);
                    telemetry.addData("Current Ball", currentBall);
                }
            } else if (pathState == 19) {
                telemetry.addData("Status", "Shooting complete!");
            }
        }
        
=======
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/pedroPathing/AutonPedroDocumentation.java
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
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if (!path2Started) {
                    // Start Path2
                    robot.startIntake(); // Start intake before aligning with set 1
                    robot.startColorSensing(); // Start color sensing for ball detection
                    startWait();
                    follower.followPath(Path2);
                    path2Started = true;
                } else if (!follower.isBusy()) {
                    // Path2 complete, start shooting sequence
                    robot.startFlywheel(); // Start flywheel early
                    robot.resetLaunchIndex(); // Reset to shoot first ball
                    setPathState(15); // Transition to shooting prep
                }
                break;
            
            case 15: // Shooting preparation
                // Keep turret aligned during preparation
                robot.updateTurret();
                // Wait for flywheel spin-up
                if (pathTimer.getElapsedTimeSeconds() >= FLYWHEEL_SPINUP_TIME) {
                    setPathState(16); // Start shooting ball 1
                }
                break;
            
            case 16: // Shoot ball 1
                if (!isWaiting) {
                    robot.updateTurret(); // Keep turret aligned
                    robot.launchOne(telemetry); // Position ball 1
                    robot.kickSpoon(); // Fire kicker (blocks ~800ms)
                    startWait(KICKER_WAIT_TIME); // Wait for kicker sequence
                    setPathState(17); // Move to ball 2
                }
                break;
            
            case 17: // Shoot ball 2
                if (!isWaiting) {
                    robot.updateTurret(); // Keep turret aligned
                    robot.launchOne(telemetry); // Position ball 2
                    robot.kickSpoon(); // Fire kicker
                    startWait(KICKER_WAIT_TIME);
                    setPathState(18); // Move to ball 3
                }
                break;
            
            case 18: // Shoot ball 3
                if (!isWaiting) {
                    robot.updateTurret(); // Keep turret aligned
                    robot.launchOne(telemetry); // Position ball 3
                    robot.kickSpoon(); // Fire kicker
                    startWait(KICKER_WAIT_TIME);
                    setPathState(19); // Move to completion
                }
                break;
            
            case 19: // Shooting complete
                if (!isWaiting) {
                    robot.stopFlywheel();
                    setPathState(2); // Continue to case 2 (or whatever comes next)
                }
                break;
            /*
            case 2:
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
        // Path 1: align - (56,8) to (56,12), linear heading 90→115
        Path1 = follower.pathBuilder()
                .setGlobalDeceleration(2.0) // Higher deceleration slows paths down significantly
                .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
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
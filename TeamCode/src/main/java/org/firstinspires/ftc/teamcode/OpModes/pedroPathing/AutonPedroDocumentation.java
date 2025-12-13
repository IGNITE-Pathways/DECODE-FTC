package org.firstinspires.ftc.teamcode.OpModes.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

@Autonomous(name = "Autonomous - Ball Collection", group = "Autonomous")
public class AutonPedroDocumentation extends OpMode {
    private Follower follower;
    private Robot robot;
    private Timer pathTimer, opmodeTimer, waitTimer;

    private int pathState;
    private boolean isWaiting = false;
    private static final double WAIT_TIME_SECONDS = 1.5; // Wait 1.5 seconds after each path
    private static final double BALL_WAIT_TIME_SECONDS = 2.0; // Longer wait for ball collection paths

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
        robot.initialize(hardwareMap, telemetry, null); // OpMode is null since we're using OpMode, not LinearOpMode
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        // You can add initialization feedback here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        robot.updateIntake(); // Update intake to keep it running
        robot.updateSpindexerSensing(); // Update color sensing for ball detection
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("is waiting", isWaiting);
        telemetry.addData("is busy", follower.isBusy());
        telemetry.addData("intake running", robot.isIntakeRunning());
        telemetry.addData("ball count", robot.getBallCount());
        telemetry.addData("ball sequence", robot.getBallSequence());
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
            // Use longer wait time for ball collection paths (states 2-4 and 7-9)
            double currentWaitTime = (pathState >= 2 && pathState <= 4) || (pathState >= 7 && pathState <= 9) 
                    ? BALL_WAIT_TIME_SECONDS : WAIT_TIME_SECONDS;
            
            if (waitTimer.getElapsedTimeSeconds() >= currentWaitTime) {
                isWaiting = false;
                waitTimer.resetTimer();
            } else {
                return; // Still waiting, don't process path states
            }
        }
        
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;
            /*case 1:
                if(!follower.isBusy()) {
                    robot.startIntake(); // Start intake before aligning with set 1
                    robot.startColorSensing(); // Start color sensing for ball detection
                    startWait();
                    follower.followPath(Path2);
                    setPathState(2);
                }
                break;
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
        waitTimer.resetTimer();
        // Store custom wait time - we'll check against WAIT_TIME_SECONDS for now
        // For ball collection, we'll use longer waits
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
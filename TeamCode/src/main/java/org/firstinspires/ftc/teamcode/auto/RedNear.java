package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Autonomous OpMode for FTC Robot
 * Converts Pedro Pathing visualizer paths into executable autonomous routine
 *
 * Routine Overview:
 * 1. Move to first ball collection zone
 * 2. Collect 3 balls in sequence
 * 3. Move to shooting position and shoot first set
 * 4. Move to second ball collection zone
 * 5. Collect 3 balls in sequence
 * 6. Move to shooting position and shoot second set
 *
 * State Machine uses PathState enum to manage path transitions
 */
@Autonomous(name = "Red Near ACTUAL", group = "Autonomous")
public class RedNear extends LinearOpMode {

    // ================== ENUMS ==================
    /**
     * Enum to track current state of autonomous routine
     * Used in state machine to manage path transitions
     */
    private enum PathState {
        GOING_TO_FIRST_SET,
        GETTING_FIRST_BALL,
        GETTING_SECOND_BALL,
        GETTING_THIRD_BALL,
        SHOOTING_FIRST_SET,
        GOING_TO_SECOND_SET,
        GETTING_TO_SECOND_SET_FIRST_BALL,
        GETTING_TO_SECOND_SET_SECOND_BALL,
        GETTING_TO_SECOND_SET_THIRD_BALL,
        SHOOTING_SECOND_SET,
        IDLE
    }

    // ================== INSTANCE VARIABLES ==================
    private Follower follower;
    private PathState pathState;
    private Paths paths;
    private ElapsedTime actionTimer;
    private ElapsedTime opModeTimer;

    // ================== CONSTANTS ==================
    // Starting pose on the field (adjust to match your starting position)
    private final Pose START_POSE = new Pose(121.530, 123.468);

    // Timing constants for actions (adjust based on your robot's speeds)
    private final double BALL_COLLECTION_TIME = 0.5; // Time to collect each ball
    private final double SHOOTING_TIME = 1.0;         // Time to shoot

    // ================== INITIALIZATION & MAIN ==================

    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry.addLine("Initializing autonomous...");
        telemetry.update();

        // Create follower instance
        follower = Constants.createFollower(hardwareMap);

        // Set starting position
        follower.setStartingPose(START_POSE);

        follower.activateAllPIDFs();

        // Build all paths
        paths = new Paths(follower);

        // Initialize state machine
        pathState = PathState.IDLE;
        actionTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();

        // Wait for user to press INIT
        telemetry.addLine("Robot ready for autonomous");
        telemetry.addLine("Press PLAY to begin");
        telemetry.addLine("Current Path State: " + pathState);
        telemetry.update();

        // Wait for start signal
        waitForStart();

        // Start timers and transition to first state
        opModeTimer.reset();
        setPathState(PathState.GOING_TO_FIRST_SET);

        // Main loop - runs until OpMode is stopped
        while (opModeIsActive()) {
            // CRITICAL: Must update follower every loop
            follower.update();

            // Execute state machine
            autonomousPathUpdate();

            // Telemetry for debugging
            telemetry.addData("Path State", pathState);
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Action Timer", actionTimer.seconds());
            telemetry.addData("Op Mode Timer", opModeTimer.seconds());
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.addData("Heading (degrees)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        // Autonomous complete
        telemetry.addLine("Autonomous routine complete!");
        telemetry.addData("Total Time", opModeTimer.seconds());
        telemetry.update();
    }

    // ================== STATE MACHINE ==================

    /**
     * Main autonomous state machine
     * Manages path transitions and robot actions
     * Called every loop iteration
     *
     * IMPORTANT: followPath() is called ONLY in setPathState(), not here
     * This method only checks conditions for state transitions
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case GOING_TO_FIRST_SET:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    setPathState(PathState.GETTING_FIRST_BALL);
                }
                break;

            case GETTING_FIRST_BALL:
                // Continue ball collection while moving
                performBallCollection();

                // Wait for path complete AND collection time
                if (!follower.isBusy() && actionTimer.seconds() > BALL_COLLECTION_TIME) {
                    setPathState(PathState.GETTING_SECOND_BALL);
                }
                break;

            case GETTING_SECOND_BALL:
                // Continue ball collection while moving
                performBallCollection();

                if (!follower.isBusy() && actionTimer.seconds() > BALL_COLLECTION_TIME) {
                    setPathState(PathState.GETTING_THIRD_BALL);
                }
                break;

            case GETTING_THIRD_BALL:
                // Continue ball collection while moving
                performBallCollection();

                if (!follower.isBusy() && actionTimer.seconds() > BALL_COLLECTION_TIME) {
                    setPathState(PathState.SHOOTING_FIRST_SET);
                }
                break;

            case SHOOTING_FIRST_SET:
                // Perform shooting action
                performShooting();

                // Wait for path complete AND shooting time
                if (!follower.isBusy() && actionTimer.seconds() > SHOOTING_TIME) {
                    setPathState(PathState.GOING_TO_SECOND_SET);
                }
                break;

            case GOING_TO_SECOND_SET:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    setPathState(PathState.GETTING_TO_SECOND_SET_FIRST_BALL);
                }
                break;

            case GETTING_TO_SECOND_SET_FIRST_BALL:
                // Continue ball collection while moving
                performBallCollection();

                if (!follower.isBusy() && actionTimer.seconds() > BALL_COLLECTION_TIME) {
                    setPathState(PathState.GETTING_TO_SECOND_SET_SECOND_BALL);
                }
                break;

            case GETTING_TO_SECOND_SET_SECOND_BALL:
                // Continue ball collection while moving
                performBallCollection();

                if (!follower.isBusy() && actionTimer.seconds() > BALL_COLLECTION_TIME) {
                    setPathState(PathState.GETTING_TO_SECOND_SET_THIRD_BALL);
                }
                break;

            case GETTING_TO_SECOND_SET_THIRD_BALL:
                // Continue ball collection while moving
                performBallCollection();

                if (!follower.isBusy() && actionTimer.seconds() > BALL_COLLECTION_TIME) {
                    setPathState(PathState.SHOOTING_SECOND_SET);
                }
                break;

            case SHOOTING_SECOND_SET:
                // Perform shooting action
                performShooting();

                // Wait for path complete AND shooting time
                if (!follower.isBusy() && actionTimer.seconds() > SHOOTING_TIME) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                // Routine complete - do nothing
                break;
        }
    }

    // ================== HELPER METHODS ==================

    /**
     * Transitions to a new path state and starts the appropriate path
     * CRITICAL: followPath() is called HERE, only once per state
     * @param newState The new PathState to transition to
     */
    private void setPathState(PathState newState) {
        pathState = newState;
        actionTimer.reset();

        // Start the appropriate path for each state
        // followPath() should only be called ONCE per state, not in the loop
        switch (newState) {
            case GOING_TO_FIRST_SET:
                follower.followPath(paths.Goingtothe1stSet);
                break;

            case GETTING_FIRST_BALL:
                follower.followPath(paths.Getting1stBall);
                break;

            case GETTING_SECOND_BALL:
                follower.followPath(paths.Getting2ndBall);
                break;

            case GETTING_THIRD_BALL:
                follower.followPath(paths.Getting3rdBall);
                break;

            case SHOOTING_FIRST_SET:
                follower.followPath(paths.Shootingthe3Ballsof1stSet);
                break;

            case GOING_TO_SECOND_SET:
                follower.followPath(paths.Goingtothe2ndSet);
                break;

            case GETTING_TO_SECOND_SET_FIRST_BALL:
                follower.followPath(paths.Gettingto1stBall);
                break;

            case GETTING_TO_SECOND_SET_SECOND_BALL:
                follower.followPath(paths.Gettingto2ndBall);
                break;

            case GETTING_TO_SECOND_SET_THIRD_BALL:
                follower.followPath(paths.Gettingto3rdBall);
                break;

            case SHOOTING_SECOND_SET:
                follower.followPath(paths.Shootingthe2ndSet);
                break;

            case IDLE:
                // No path to follow
                break;
        }
    }

    /**
     * Placeholder for ball collection action
     * TODO: Replace with actual intake/collection mechanism control
     * This would include:
     * - Starting intake motor
     * - Positioning intake arm
     * - Running continuously while collecting
     */
    private void performBallCollection() {
        // TODO: Implement actual ball collection logic
        // Example:
        // intakeMotor.setPower(1.0);
        // intakeArm.setPosition(COLLECTION_POSITION);
    }

    /**
     * Placeholder for shooting action
     * TODO: Replace with actual shooting mechanism control
     * This would include:
     * - Starting flywheel
     * - Positioning angle servo
     * - Firing mechanism
     */
    private void performShooting() {
        // TODO: Implement actual shooting logic
        // Example:
        // flywheel.setPower(1.0);
        // angleServo.setPosition(SHOOTING_ANGLE);
        // fireMechanism.setPosition(FIRE_POSITION);
    }

    // ================== PATH DEFINITIONS ==================

    /**
     * Inner class containing all PathChain definitions
     * Each path is built using Pedro Pathing's path builder
     */
    public static class Paths {
        // First set paths
        public PathChain Goingtothe1stSet;
        public PathChain Getting1stBall;
        public PathChain Getting2ndBall;
        public PathChain Getting3rdBall;
        public PathChain Shootingthe3Ballsof1stSet;

        // Second set paths
        public PathChain Goingtothe2ndSet;
        public PathChain Gettingto1stBall;
        public PathChain Gettingto2ndBall;
        public PathChain Gettingto3rdBall;
        public PathChain Shootingthe2ndSet;

        /**
         * Constructor builds all paths
         * @param follower The Follower instance to use for path building
         */
        public Paths(Follower follower) {
            // ===== FIRST SET PATHS =====

            Goingtothe1stSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(121.530, 123.468),
                            new Pose(91.415, 97.825),
                            new Pose(100.611, 83.810)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(307), Math.toRadians(0))
                    .build();

            Getting1stBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(100.611, 83.810),
                            new Pose(107.896, 83.869)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Getting2ndBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(107.896, 83.869),
                            new Pose(113.508, 83.896)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Getting3rdBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(113.508, 83.896),
                            new Pose(118.907, 83.914)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shootingthe3Ballsof1stSet = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(118.907, 83.914),
                            new Pose(96.112, 95.741)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // ===== SECOND SET PATHS =====

            Goingtothe2ndSet = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96.112, 95.741),
                            new Pose(101.753, 59.633)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Gettingto1stBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(101.753, 59.633),
                            new Pose(107.656, 59.674)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Gettingto2ndBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(107.656, 59.674),
                            new Pose(112.825, 59.751)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Gettingto3rdBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(112.825, 59.751),
                            new Pose(117.782, 59.778)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shootingthe2ndSet = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(117.782, 59.778),
                            new Pose(82.983, 80.719)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
}
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Autonomous OpMode for FTC Ball Pickup and Shooting Routine
 * Uses Pedro Pathing for path following with a finite state machine
 *
 * This routine:
 * 1. Moves to the 1st set of balls
 * 2. Collects 3 balls from the 1st set
 * 3. Returns and shoots the 3 balls
 * 4. Moves to the 2nd set of balls
 * 5. Collects 3 balls from the 2nd set
 * 6. Moves to shooting position and scores
 */
@Autonomous(name = "Auto - Ball Pickup (Pedro Pathing)", group = "Autonomous")
public class BlueNear extends OpMode {

    // Follower for path following
    private Follower follower;

    // Path chains for autonomous routine
    private PathChain goingToThe1stSet, getting1stBall, getting2ndBall, getting3rdBall;
    private PathChain shootingThe3BallsOf1stSet;
    private PathChain goingToThe2ndSet, gettingTo1stBall, gettingTo2ndBall, gettingTo3rdBall;
    private PathChain shootingThe2ndSet;

    // Timers
    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime opmodeTimer = new ElapsedTime();

    // State machine for autonomous progression
    private int pathState;

    // Starting pose for the robot
    private final Pose START_POSE = new Pose(24.143, 125.525, Math.toRadians(233));

    // Shooting wait times (in seconds)
    private static final double FIRST_SHOOT_TIME = 1.5;
    private static final double SECOND_SHOOT_TIME = 2.0;

    private static final double PATH_SPEED = 0.45;


    /**
     * Initialize the OpMode - called when INIT is pressed
     */
    @Override
    public void init() {
        // Initialize pathState
        pathState = 0;

        // Initialize the follower with the hardware map
        follower = Constants.createFollower(hardwareMap);

        // Set the starting pose for localization
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(PATH_SPEED);

        // Build all paths
        buildPaths();

        // Send telemetry data to driver station
        telemetry.addLine("Initialization complete!");
        telemetry.addLine("Starting Pose: " + START_POSE);
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    /**
     * Build all paths for the autonomous routine
     */
    private void buildPaths() {
        // PATH 1: Go to the 1st set of balls
        goingToThe1stSet = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.143, 125.525),
                                new Pose(48.215, 96.000),
                                new Pose(41.794, 84.215)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(233), Math.toRadians(180))
                .build();

        // PATH 2: Get 1st ball from 1st set
        getting1stBall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(41.794, 84.215),
                                new Pose(36.099, 84.072)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // PATH 3: Get 2nd ball from 1st set
        getting2ndBall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(36.099, 84.072),
                                new Pose(30.556, 84.099)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // PATH 4: Get 3rd ball from 1st set
        getting3rdBall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(30.556, 84.099),
                                new Pose(25.408, 84.117)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // PATH 5: Return to shooting position for 1st set
        shootingThe3BallsOf1stSet = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.408, 84.117),
                                new Pose(48.045, 96.350)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // PATH 6: Move to the 2nd set of balls
        goingToThe2ndSet = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(48.045, 96.350),
                                new Pose(41.314, 60.444)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // PATH 7: Get 1st ball from 2nd set
        gettingTo1stBall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(41.314, 60.444),
                                new Pose(36.265, 60.283)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // PATH 8: Get 2nd ball from 2nd set
        gettingTo2ndBall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(36.265, 60.283),
                                new Pose(31.090, 60.157)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // PATH 9: Get 3rd ball from 2nd set
        gettingTo3rdBall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(31.090, 60.157),
                                new Pose(25.906, 60.184)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // PATH 10: Move to final shooting position
        shootingThe2ndSet = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.906, 60.184),
                                new Pose(60.673, 81.327)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    /**
     * Start the OpMode - called when START is pressed
     */
    @Override
    public void start() {
        // Reset timers
        pathTimer.reset();
        opmodeTimer.reset();

        // Start the first path state
        setPathState(0);
    }

    /**
     * Main loop for the autonomous routine
     */
    @Override
    public void loop() {
        // Update the follower position estimate
        follower.update();

        // State machine for path progression
        autonomousPathUpdate();

        // Telemetry for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("OpMode Time", opmodeTimer.seconds());
        telemetry.addData("Robot Position", follower.getPose());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    /**
     * Finite State Machine for managing autonomous path progression
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to the 1st set of balls
                follower.followPath(goingToThe1stSet);
                setPathState(1);
                break;

            case 1:
                // Wait until path is complete, then start collecting 1st ball
                if (!follower.isBusy()) {
                    follower.followPath(getting1stBall);
                    setPathState(2);
                    // TODO: Activate intake mechanism
                    // Example: intake.setPower(1.0);
                }
                break;

            case 2:
                // Wait for 1st ball collection, then move to 2nd ball
                if (!follower.isBusy()) {
                    follower.followPath(getting2ndBall);
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for 2nd ball collection, then move to 3rd ball
                if (!follower.isBusy()) {
                    follower.followPath(getting3rdBall);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for 3rd ball collection, then move to shooting position
                if (!follower.isBusy()) {
                    follower.followPath(shootingThe3BallsOf1stSet);
                    setPathState(5);
                    // TODO: Deactivate intake mechanism
                    // Example: intake.setPower(0.0);
                }
                break;

            case 5:
                // Wait until we reach shooting position
                if (!follower.isBusy()) {
                    // Start shooting sequence
                    setPathState(6);
                    // TODO: Activate shooter mechanism
                    // Example: shooter.spin();
                }
                break;

            case 6:
                // Wait for shooting to complete
                if (pathTimer.seconds() > FIRST_SHOOT_TIME) {
                    // TODO: Stop shooter
                    // Example: shooter.stop();

                    // Move to 2nd set of balls
                    follower.followPath(goingToThe2ndSet);
                    setPathState(7);
                }
                break;

            case 7:
                // Wait until we reach 2nd set, then start collecting
                if (!follower.isBusy()) {
                    follower.followPath(gettingTo1stBall);
                    setPathState(8);
                    // TODO: Activate intake mechanism again
                    // Example: intake.setPower(1.0);
                }
                break;

            case 8:
                // Wait for 1st ball from 2nd set, then move to 2nd ball
                if (!follower.isBusy()) {
                    follower.followPath(gettingTo2ndBall);
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for 2nd ball from 2nd set, then move to 3rd ball
                if (!follower.isBusy()) {
                    follower.followPath(gettingTo3rdBall);
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for 3rd ball from 2nd set, then move to final shooting position
                if (!follower.isBusy()) {
                    follower.followPath(shootingThe2ndSet);
                    setPathState(11);
                    // TODO: Deactivate intake
                    // Example: intake.setPower(0.0);
                }
                break;

            case 11:
                // Wait until we reach final shooting position
                if (!follower.isBusy()) {
                    // Start final shooting sequence
                    setPathState(12);
                    // TODO: Activate final shooter
                    // Example: shooter.spin();
                }
                break;

            case 12:
                // Wait for final shooting to complete
                if (pathTimer.seconds() > SECOND_SHOOT_TIME) {
                    // TODO: Stop shooter
                    // Example: shooter.stop();

                    // Autonomous complete
                    setPathState(13);
                }
                break;

            case 13:
                // Autonomous routine complete - do nothing
                telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                telemetry.addData("Total Time", opmodeTimer.seconds());
                break;

            default:
                // Handle unexpected states
                telemetry.addLine("ERROR: Unknown state " + pathState);
                requestOpModeStop();
                break;
        }
    }

    /**
     * Helper method to transition between path states and reset the path timer
     */
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.reset();
    }
}
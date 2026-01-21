package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * FTC Autonomous OpMode using Pedro Pathing
 *
 * Executes a two-cycle ball collection routine:
 * Cycle 1: GoToNearestBalls → GetBall1 → GetBall2 → GetBall3 → ReturnToShoot
 * Cycle 2: GoToNextSet → GetBall1 → GetBall2 → GetBall3 → ReturnToShoot
 *
 * Uses LinearOpMode with sequential execution for clearer autonomous logic.
 */
@Autonomous(name = "Red Far", group = "Autonomous")
public class RedFar extends OpMode {

    private Follower follower;
    private int pathState;
    private PathChain goingToNearestBalls, gettingFirstBallCycle1, gettingSecondBallCycle1,
            gettingThirdBallCycle1, goingBackToShoot1, gettingNextSetOfBalls,
            gettingFirstBallCycle2, gettingSecondBallCycle2, gettingThirdBallCycle2,
            goingBackToShoot2;

    // Starting pose
    private final Pose startPose = new Pose(84.652, 9.071, Math.toRadians(0));

    // TODO: Add your subsystem objects here (shooter, intake, spindexer, etc.)
    // private ShooterSubsystem shooter;
    // private IntakeSubsystem intake;
    // private SpindexerSubsystem spindexer;

    @Override
    public void init() {
        // Initialize Pedro Pathing follower using Constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        // Activate all PIDFs so the tuned PIDF constants are used during following
        follower.activateAllPIDFs();

        // Build all paths (positions unchanged from original RedFar)
        buildPaths();

        // Set up telemetry
        telemetry.addLine("Pedro Pathing Autonomous Initialized!");
        telemetry.addLine("Starting pose: (84.652, 9.071, 0°)");
        telemetry.addData("Status", "Ready - waiting for start");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Optional: add pre-start telemetry or checks here if needed
    }

    @Override
    public void start() {
        // Start with path state 0 when OpMode is started
        pathState = 0;
    }

    @Override
    public void loop() {
        // Update follower and advance the autonomous state machine
        follower.update();
        autonomousPathUpdate();

        // Update telemetry with current position
        Pose current = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", current.getX());
        telemetry.addData("Y", current.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(current.getHeading()));
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.update();
    }

    /**
     * State machine to manage path progression
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Go to nearest balls
                follower.followPath(goingToNearestBalls);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    // TODO: Start intake
                    // intake.start();
                    follower.followPath(gettingFirstBallCycle1);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(gettingSecondBallCycle1);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(gettingThirdBallCycle1);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    // TODO: Stop intake
                    // intake.stop();
                    follower.followPath(goingBackToShoot1);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // TODO: Shoot balls
                    // shooter.shoot();
                    // sleep(2000); // Wait for balls to shoot
                    follower.followPath(gettingNextSetOfBalls);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    // TODO: Start intake
                    // intake.start();
                    follower.followPath(gettingFirstBallCycle2);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(gettingSecondBallCycle2);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(gettingThirdBallCycle2);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    // TODO: Stop intake
                    // intake.stop();
                    follower.followPath(goingBackToShoot2);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    // TODO: Shoot balls
                    // shooter.shoot();
                    // sleep(2000); // Wait for balls to shoot
                    pathState = 11; // Done
                }
                break;
            case 11:
                // Autonomous complete
                break;
        }
    }

    /**
     * Build all PathChains for the autonomous routine
     * Extracted from your Pedro Pathing Visualizer
     */
    private void buildPaths() {
        // ===== CYCLE 1: First Set of Balls =====

        // Define intermediate poses for cleaner code
        Pose cycle1Ball1Start = new Pose(101.559, 35.453, Math.toRadians(0));
        Pose cycle1Ball1End = new Pose(107.779, 35.511, Math.toRadians(0));
        Pose cycle1Ball2End = new Pose(113.745, 35.511, Math.toRadians(0));
        Pose cycle1Ball3End = new Pose(119.649, 35.511, Math.toRadians(0));

        goingToNearestBalls = follower.pathBuilder()
                .addPath(new BezierLine(startPose, cycle1Ball1Start))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gettingFirstBallCycle1 = follower.pathBuilder()
                .addPath(new BezierLine(cycle1Ball1Start, cycle1Ball1End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gettingSecondBallCycle1 = follower.pathBuilder()
                .addPath(new BezierLine(cycle1Ball1End, cycle1Ball2End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gettingThirdBallCycle1 = follower.pathBuilder()
                .addPath(new BezierLine(cycle1Ball2End, cycle1Ball3End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goingBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(cycle1Ball3End, startPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // ===== CYCLE 2: Second Set of Balls =====

        // Define intermediate poses
        Pose cycle2ControlPoint = new Pose(96.735, 54.372, Math.toRadians(0));
        Pose cycle2Ball1Start = new Pose(101.300, 59.831, Math.toRadians(0));
        Pose cycle2Ball1End = new Pose(108.322, 59.923, Math.toRadians(0));
        Pose cycle2Ball2End = new Pose(114.068, 59.780, Math.toRadians(0));
        Pose cycle2Ball3End = new Pose(119.792, 59.815, Math.toRadians(0));
        Pose cycle2ShootEnd = new Pose(84.581, 9.000, Math.toRadians(0));

        gettingNextSetOfBalls = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, cycle2ControlPoint, cycle2Ball1Start))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gettingFirstBallCycle2 = follower.pathBuilder()
                .addPath(new BezierLine(cycle2Ball1Start, cycle2Ball1End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gettingSecondBallCycle2 = follower.pathBuilder()
                .addPath(new BezierLine(cycle2Ball1End, cycle2Ball2End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gettingThirdBallCycle2 = follower.pathBuilder()
                .addPath(new BezierLine(cycle2Ball2End, cycle2Ball3End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goingBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(cycle2Ball3End, cycle2ShootEnd))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
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

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Combined Autonomous OpMode for Ball Collection Routine
 *
 * This autonomous routine:
 * 1. Shoots 3 preloaded balls (6 second wait)
 * 2. Collects 3 balls from the first zone
 * 3. Returns to shoot position and shoots
 * 4. Collects 3 balls from the second zone
 * 5. Returns to shoot position and shoots
 * 6. Collects 3 balls from the third zone
 * 7. Moves to final position
 */
@Autonomous(name = "Blue Far Combined", group = "Autonomous")
public class BlueFarImprovedArjun2 extends OpMode {

    // ==================== SHOOTING CONSTANTS ====================
    // 10ft preset for shooting
    private static final double FLYWHEEL_POWER = 0.67;
    private static final double HOOD_POSITION = 0.70;
    private static final double SHOOT_TIME_SECONDS = 6.0;

    // Turret locked position
    private static final double TURRET_LOCKED_POSITION = 0.47;

    // Path speed (45%)
    private static final double PATH_SPEED = 0.45;

    // Path timeout (15 seconds per path)
    private static final double PATH_TIMEOUT = 15.0;

    // ==================== ROBOT COMPONENTS ====================
    private Follower follower;
    private Paths paths;

    // Robot components
    private IntakeTransfer intakeTransfer;
    private Launcher launcher;
    private Servo turretServo;

    // State Management
    private PathState pathState = PathState.PRELOAD_SHOOT_SETUP;
    private Timer pathTimer;
    private Timer opModeTimer;
    private Timer shootTimer;
    private double currentSpeed = PATH_SPEED;  // Track current speed for telemetry

    /**
     * Path State Enumeration
     * Defines all possible states in the autonomous routine
     */
    public enum PathState {
        // Preload shooting
        PRELOAD_SHOOT_SETUP,
        PRELOAD_SHOOTING,

        // First Ball Set (Starting Zone)
        GOING_TO_NEAREST_BALLS,
        GETTING_FIRST_BALL_SET_1,
        GETTING_SECOND_BALL_SET_1,
        GETTING_THIRD_BALL_SET_1,
        GOING_BACK_TO_SHOOT_SET_1,
        SHOOTING_SET_1,

        // Second Ball Set
        GETTING_NEXT_SET_OF_BALLS,
        GETTING_FIRST_BALL_SET_2,
        GETTING_SECOND_BALL_SET_2,
        GETTING_THIRD_BALL_SET_2,
        GOING_BACK_TO_SHOOT_SET_2,
        SHOOTING_SET_2,

        // Third Ball Set
        GETTING_THIRD_SET_OF_BALLS,
        GETTING_FIRST_BALL_SET_3,
        GETTING_SECOND_BALL_SET_3,
        GETTING_THIRD_BALL_SET_3,
        GOING_TO_FINAL_POSITION,

        IDLE
    }

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        // Initialize robot components
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize turret servo
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            turretServo.setPosition(TURRET_LOCKED_POSITION);
            telemetry.addLine("Turret Servo: OK");
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("Turret Servo: NOT FOUND");
        }

        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60.845, 7.910, Math.toRadians(180)));
        follower.setMaxPower(PATH_SPEED);

        // Create path objects
        paths = new Paths(follower);

        telemetry.addLine("Blue Far Combined Auto Initialized");
        telemetry.addData("Path Speed", "%.0f%%", PATH_SPEED * 100);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep turret locked during init
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        shootTimer.resetTimer();
        pathState = PathState.PRELOAD_SHOOT_SETUP;
    }

    @Override
    public void loop() {
        // CRITICAL: Update follower and launcher every loop
        follower.update();
        launcher.update();

        // Keep turret locked
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }

        // Execute current path state
        autonomousPathUpdate();

        // Telemetry
        updateTelemetry();
    }

    /**
     * State machine for autonomous path execution
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            // ========== PRELOAD SHOOTING ==========
            case PRELOAD_SHOOT_SETUP:
                // Set up shooter at 10ft preset
                launcher.setPower(FLYWHEEL_POWER);
                launcher.setHoodPosition(HOOD_POSITION);
                launcher.setSpinning(true);

                // Ramp up and start feeding balls
                intakeTransfer.transferUp();
                intakeTransfer.startIntake();

                shootTimer.resetTimer();
                setPathState(PathState.PRELOAD_SHOOTING);
                break;

            case PRELOAD_SHOOTING:
                // Keep flywheel spinning during wait
                if (launcher.flyWheelMotor != null) {
                    launcher.flyWheelMotor.setPower(FLYWHEEL_POWER);
                }
                if (launcher.flyWheelMotor2 != null) {
                    launcher.flyWheelMotor2.setPower(FLYWHEEL_POWER);
                }
                launcher.setHoodPosition(HOOD_POSITION);
                launcher.setSpinning(true);

                // Pulsed feeding with ramp cycling to let flywheel recover
                double elapsed = shootTimer.getElapsedTimeSeconds();

                // Spin-up period (0-1.5s) - just flywheel, no feeding
                if (elapsed < 1.5) {
                    intakeTransfer.stopIntake();
                    intakeTransfer.transferDown();
                }
                // Feeding period with pauses
                else {
                    long ms = (long) ((elapsed - 1.5) * 1000);
                    long feedCycle = ms % 800;  // 800ms cycle (500 feed, 300 pause)
                    if (feedCycle < 500) {
                        // Feed phase - ramp up, intake on
                        intakeTransfer.transferUp();
                        intakeTransfer.startIntake();
                    } else {
                        // Pause phase - ramp down, intake off (let flywheel recover)
                        intakeTransfer.stopIntake();
                        intakeTransfer.transferDown();
                    }
                }

                // Wait for preload shoot to complete
                if (elapsed >= SHOOT_TIME_SECONDS) {
                    stopShooting();
                    // Start collecting first ball set with intake running, SLOW SPEED
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);  // Slow speed for collection
                    intakeTransfer.startIntake();
                    follower.followPath(paths.goingToNearestBalls);
                    setPathState(PathState.GOING_TO_NEAREST_BALLS);
                }
                break;

            // ========== FIRST BALL SET ==========
            case GOING_TO_NEAREST_BALLS:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingFirstBallSet1);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_1);
                }
                break;

            case GETTING_FIRST_BALL_SET_1:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingSecondBallSet1);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_1);
                }
                break;

            case GETTING_SECOND_BALL_SET_1:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingThirdBallSet1);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_1);
                }
                break;

            case GETTING_THIRD_BALL_SET_1:
                // Keep intake running
                if (!follower.isBusy()) {
                    // Stop intake for return path, GO MAX SPEED
                    intakeTransfer.stopIntake();
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);  // Full speed to shooting position
                    follower.followPath(paths.goingBackToShootSet1);
                    setPathState(PathState.GOING_BACK_TO_SHOOT_SET_1);
                }
                break;

            case GOING_BACK_TO_SHOOT_SET_1:
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(PathState.SHOOTING_SET_1);
                }
                break;

            case SHOOTING_SET_1:
                performShooting();
                if (shootTimer.getElapsedTimeSeconds() >= SHOOT_TIME_SECONDS) {
                    stopShooting();
                    // Start collecting second ball set with intake running, SLOW SPEED
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);  // Back to slow speed for collection
                    intakeTransfer.startIntake();
                    follower.followPath(paths.gettingNextSetOfBalls);
                    setPathState(PathState.GETTING_NEXT_SET_OF_BALLS);
                }
                break;

            // ========== SECOND BALL SET ==========
            case GETTING_NEXT_SET_OF_BALLS:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingFirstBallSet2);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_2);
                }
                break;

            case GETTING_FIRST_BALL_SET_2:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingSecondBallSet2);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_2);
                }
                break;

            case GETTING_SECOND_BALL_SET_2:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingThirdBallSet2);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_2);
                }
                break;

            case GETTING_THIRD_BALL_SET_2:
                // Keep intake running
                if (!follower.isBusy()) {
                    // Stop intake for return path, GO MAX SPEED
                    intakeTransfer.stopIntake();
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);  // Full speed to shooting position
                    follower.followPath(paths.goingBackToShootSet2);
                    setPathState(PathState.GOING_BACK_TO_SHOOT_SET_2);
                }
                break;

            case GOING_BACK_TO_SHOOT_SET_2:
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(PathState.SHOOTING_SET_2);
                }
                break;

            case SHOOTING_SET_2:
                performShooting();
                if (shootTimer.getElapsedTimeSeconds() >= SHOOT_TIME_SECONDS) {
                    stopShooting();
                    // Start collecting third ball set with intake running, SLOW SPEED
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);  // Back to slow speed for collection
                    intakeTransfer.startIntake();
                    follower.followPath(paths.gettingThirdSetOfBalls);
                    setPathState(PathState.GETTING_THIRD_SET_OF_BALLS);
                }
                break;

            // ========== THIRD BALL SET ==========
            case GETTING_THIRD_SET_OF_BALLS:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingFirstBallSet3);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_3);
                }
                break;

            case GETTING_FIRST_BALL_SET_3:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingSecondBallSet3);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_3);
                }
                break;

            case GETTING_SECOND_BALL_SET_3:
                // Keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingThirdBallSet3);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_3);
                }
                break;

            case GETTING_THIRD_BALL_SET_3:
                // Keep intake running
                if (!follower.isBusy()) {
                    // Stop intake for final path, GO MAX SPEED
                    intakeTransfer.stopIntake();
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);  // Full speed to final position
                    follower.followPath(paths.path15);
                    setPathState(PathState.GOING_TO_FINAL_POSITION);
                }
                break;

            case GOING_TO_FINAL_POSITION:
                if (!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                // Autonomous complete
                break;
        }

        // Safety timeout - prevent infinite loops
        if (pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT && pathState != PathState.IDLE) {
            telemetry.addLine("WARNING: Path timeout exceeded!");
            setPathState(PathState.IDLE);
        }
    }

    /**
     * Start shooting sequence
     */
    private void startShooting() {
        launcher.setPower(FLYWHEEL_POWER);
        launcher.setHoodPosition(HOOD_POSITION);
        launcher.setSpinning(true);
        intakeTransfer.transferUp();
        intakeTransfer.startIntake();
        shootTimer.resetTimer();
    }

    /**
     * Perform shooting with pulsed feeding
     */
    private void performShooting() {
        // Keep flywheel spinning
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(FLYWHEEL_POWER);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(FLYWHEEL_POWER);
        }
        launcher.setHoodPosition(HOOD_POSITION);
        launcher.setSpinning(true);

        double elapsed = shootTimer.getElapsedTimeSeconds();

        // Spin-up period (0-1.5s) - just flywheel, no feeding
        if (elapsed < 1.5) {
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
        }
        // Feeding period with pauses
        else {
            long ms = (long) ((elapsed - 1.5) * 1000);
            long feedCycle = ms % 800;  // 800ms cycle (500 feed, 300 pause)
            if (feedCycle < 500) {
                // Feed phase - ramp up, intake on
                intakeTransfer.transferUp();
                intakeTransfer.startIntake();
            } else {
                // Pause phase - ramp down, intake off (let flywheel recover)
                intakeTransfer.stopIntake();
                intakeTransfer.transferDown();
            }
        }
    }

    /**
     * Stop shooting and turn off systems
     */
    private void stopShooting() {
        launcher.setSpinning(false);
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(0);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(0);
        }
        intakeTransfer.stopIntake();
        intakeTransfer.transferDown();
    }

    /**
     * Helper method to transition between path states
     */
    private void setPathState(PathState newState) {
        if (pathState != newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }
    }

    /**
     * Update telemetry with robot state and debugging information
     */
    private void updateTelemetry() {
        telemetry.addLine("=== Autonomous Status ===");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current Speed", String.format("%.0f%%", currentSpeed * 100));
        telemetry.addData("Path Time (s)", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("OpMode Time (s)", String.format("%.2f", opModeTimer.getElapsedTimeSeconds()));

        telemetry.addLine();
        telemetry.addLine("=== Shooter Status ===");
        telemetry.addData("Flywheel", launcher.isSpinning() ? "ON" : "OFF");
        telemetry.addData("Hood Position", String.format("%.2f", launcher.getHoodPosition()));
        telemetry.addData("Turret", String.format("%.2f (locked)", TURRET_LOCKED_POSITION));

        telemetry.addLine();
        telemetry.addLine("=== Robot Position ===");
        Pose currentPose = follower.getPose();
        telemetry.addData("X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(currentPose.getHeading())));

        telemetry.addLine();
        if (opModeTimer.getElapsedTimeSeconds() > 27) {
            telemetry.addLine("WARNING: Approaching 30-second autonomous timeout!");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Turn off all systems
        if (launcher != null) {
            launcher.setSpinning(false);
        }
        if (intakeTransfer != null) {
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
        }
        follower.breakFollowing();
    }

    /**
     * ========================================
     * PATH DEFINITIONS - Inner Class
     * ========================================
     *
     * All paths from BlueFarImprovedArjun2
     */
    public static class Paths {
        // First Ball Set
        public PathChain goingToNearestBalls;
        public PathChain gettingFirstBallSet1;
        public PathChain gettingSecondBallSet1;
        public PathChain gettingThirdBallSet1;
        public PathChain goingBackToShootSet1;

        // Second Ball Set
        public PathChain gettingNextSetOfBalls;
        public PathChain gettingFirstBallSet2;
        public PathChain gettingSecondBallSet2;
        public PathChain gettingThirdBallSet2;
        public PathChain goingBackToShootSet2;

        // Third Ball Set
        public PathChain gettingThirdSetOfBalls;
        public PathChain gettingFirstBallSet3;
        public PathChain gettingSecondBallSet3;
        public PathChain gettingThirdBallSet3;
        public PathChain path15;

        /**
         * Constructor - Build all paths
         */
        public Paths(Follower follower) {
            // ========== FIRST BALL SET PATHS ==========

            goingToNearestBalls = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(60.845, 7.910),
                            new Pose(41.172, 36.034)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gettingFirstBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.172, 36.034),
                            new Pose(35.392, 35.899)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingSecondBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(35.392, 35.899),
                            new Pose(29.358, 35.899)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingThirdBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(29.358, 35.899),
                            new Pose(22.487, 35.899)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            goingBackToShootSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.487, 35.899),
                            new Pose(60.845, 7.910)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // ========== SECOND BALL SET PATHS ==========

            gettingNextSetOfBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.845, 7.910),
                            new Pose(40.606, 45.856),
                            new Pose(39.752, 59.831)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gettingFirstBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(39.752, 59.831),
                            new Pose(34.580, 59.730)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingSecondBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34.580, 59.730),
                            new Pose(30.068, 59.780)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingThirdBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(30.068, 59.780),
                            new Pose(24.566, 60.008)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            goingBackToShootSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.566, 60.008),
                            new Pose(60.845, 7.910)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // ========== THIRD BALL SET PATHS ==========

            gettingThirdSetOfBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.845, 7.910),
                            new Pose(41.666, 57.846),
                            new Pose(40.158, 84.169)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gettingFirstBallSet3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(40.158, 84.169),
                            new Pose(34.885, 84.169)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gettingSecondBallSet3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34.885, 84.169),
                            new Pose(30.070, 84.118)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingThirdBallSet3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(30.070, 84.118),
                            new Pose(24.513, 83.989)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            path15 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(24.513, 83.989),
                            new Pose(32.543, 92.571),
                            new Pose(40.994, 101.603)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}
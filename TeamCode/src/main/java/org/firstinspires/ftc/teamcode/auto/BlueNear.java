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

import org.firstinspires.ftc.teamcode.auto.Pedro.Constants;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Combined Autonomous OpMode for Ball Collection Routine
 */
@Autonomous(name = "Blue Near", group = "Autonomous")
public class BlueNear extends OpMode {

    // ==================== CLASS FIELDS ====================
    private ShootingFunction.Configuration preloadConfig;
    private ShootingFunction.Configuration set1Config;
    private ShootingFunction.Configuration set2Config;
    private ShootingFunction.Configuration activeConfig;

    // ==================== PATH CONFIGURATION ====================
    private static final double PATH_SPEED = 0.45;
    private static final double PATH_TIMEOUT = 15.0;

    // ==================== POSE CONSTANTS ====================
    private static final double HEADING_233 = Math.toRadians(233);
    private static final double HEADING_180 = Math.toRadians(180);
    private static final Pose START_POSE = new Pose(24.143, 125.525, HEADING_233);
    private static final Pose SHOOT_POSE = new Pose(48.045, 96.350);
    private static final Pose SPIKE1_APPROACH = new Pose(41.794, 84.215);
    private static final Pose SPIKE1_BALL1 = new Pose(36.099, 84.072);
    private static final Pose SPIKE1_BALL2 = new Pose(30.556, 84.099);
    private static final Pose SPIKE1_BALL3 = new Pose(25.408, 84.117);
    private static final Pose SPIKE2_APPROACH = new Pose(41.314, 63.444);
    private static final Pose SPIKE2_BALL1 = new Pose(36.265, 63.283);
    private static final Pose SPIKE2_BALL2 = new Pose(31.090, 63.157);
    private static final Pose SPIKE2_BALL3 = new Pose(25.906, 63.184);
    private static final Pose FINAL_SHOOT_POSE = new Pose(60.673, 81.327);
    private static final Pose SPIKE1_CURVE_CONTROL = new Pose(48.215, 96.000);
    private static final Pose SPIKE2_CURVE_CONTROL = new Pose(55.517695550602824, 63.52260948954108);

    // ==================== ROBOT COMPONENTS ====================
    private Follower follower;
    private Paths paths;
    private IntakeTransfer intakeTransfer;
    private Launcher launcher;
    private Servo turretServo;

    // ==================== STATE MANAGEMENT ====================
    private PathState pathState = PathState.PRELOAD_SHOOT_SETUP;
    private Timer pathTimer;
    private Timer opModeTimer;
    private Timer shootTimer;
    private double currentSpeed = PATH_SPEED;

    public enum PathState {
        PRELOAD_SHOOT_SETUP, PRELOAD_SHOOTING,
        GOING_TO_NEAREST_BALLS, GETTING_FIRST_BALL_SET_1, GETTING_SECOND_BALL_SET_1, GETTING_THIRD_BALL_SET_1,
        GOING_BACK_TO_SHOOT_SET_1, SHOOTING_SET_1,
        GETTING_NEXT_SET_OF_BALLS, GETTING_FIRST_BALL_SET_2, GETTING_SECOND_BALL_SET_2, GETTING_THIRD_BALL_SET_2,
        GOING_BACK_TO_SHOOT_SET_2, SHOOTING_SET_2,
        IDLE
    }

    @Override
    public void init() {
        // Load shooting configurations
        preloadConfig = ShootingFunction.getConfiguration(
                ShootingFunction.AutonPath.BLUE_NEAR,
                ShootingFunction.ShootingPosition.PRELOAD
        );
        set1Config = ShootingFunction.getConfiguration(
                ShootingFunction.AutonPath.BLUE_NEAR,
                ShootingFunction.ShootingPosition.SET_1
        );
        set2Config = ShootingFunction.getConfiguration(
                ShootingFunction.AutonPath.BLUE_NEAR,
                ShootingFunction.ShootingPosition.SET_2
        );

        // Initialize timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        // Initialize subsystems
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize turret servo
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            telemetry.addLine("Turret Servo: OK");
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("Turret Servo: NOT FOUND");
        }

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(PATH_SPEED);
        follower.activateAllPIDFs();
        paths = new Paths(follower);

        telemetry.addLine("Blue Near Combined Auto Initialized");
        telemetry.addData("Path Speed", "%.0f%%", PATH_SPEED * 100);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Turret is now controlled by activeConfig in main loop
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
        follower.update();
        launcher.update();

        // Update turret to active configuration
        if (turretServo != null && activeConfig != null) {
            turretServo.setPosition(activeConfig.turretPosition);
        }

        autonomousPathUpdate();
        updateTelemetry();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_SHOOT_SETUP:
                startShooting(preloadConfig);
                setPathState(PathState.PRELOAD_SHOOTING);
                break;

            case PRELOAD_SHOOTING:
                ShootingFunction.performShooting(launcher, intakeTransfer, shootTimer, activeConfig);

                double elapsed = shootTimer.getElapsedTimeSeconds();
                if (elapsed >= activeConfig.shootTimeSeconds) {
                    stopShooting();
                    currentSpeed = 0.8;
                    follower.setMaxPower(0.8);
                    intakeTransfer.startIntake();
                    follower.followPath(paths.goingToNearestBalls);
                    setPathState(PathState.GOING_TO_NEAREST_BALLS);
                }
                break;

            case GOING_TO_NEAREST_BALLS:
                if (!follower.isBusy()) {
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);
                    follower.followPath(paths.gettingFirstBallSet1);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_1);
                }
                break;

            case GETTING_FIRST_BALL_SET_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingSecondBallSet1);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_1);
                }
                break;

            case GETTING_SECOND_BALL_SET_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingThirdBallSet1);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_1);
                }
                break;

            case GETTING_THIRD_BALL_SET_1:
                if (!follower.isBusy()) {
                    intakeTransfer.stopIntake();
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.goingBackToShootSet1);
                    setPathState(PathState.GOING_BACK_TO_SHOOT_SET_1);
                }
                break;

            case GOING_BACK_TO_SHOOT_SET_1:
                if (!follower.isBusy()) {
                    startShooting(set1Config);
                    setPathState(PathState.SHOOTING_SET_1);
                }
                break;

            case SHOOTING_SET_1:
                ShootingFunction.performShooting(launcher, intakeTransfer, shootTimer, activeConfig);

                if (shootTimer.getElapsedTimeSeconds() >= activeConfig.shootTimeSeconds) {
                    stopShooting();
                    currentSpeed = 0.8;
                    follower.setMaxPower(0.8);
                    intakeTransfer.startIntake();
                    follower.followPath(paths.gettingNextSetOfBalls);
                    setPathState(PathState.GETTING_NEXT_SET_OF_BALLS);
                }
                break;

            case GETTING_NEXT_SET_OF_BALLS:
                if (!follower.isBusy()) {
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);
                    follower.followPath(paths.gettingFirstBallSet2);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_2);
                }
                break;

            case GETTING_FIRST_BALL_SET_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingSecondBallSet2);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_2);
                }
                break;

            case GETTING_SECOND_BALL_SET_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.gettingThirdBallSet2);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_2);
                }
                break;

            case GETTING_THIRD_BALL_SET_2:
                if (!follower.isBusy()) {
                    intakeTransfer.stopIntake();
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.goingBackToShootSet2);
                    setPathState(PathState.GOING_BACK_TO_SHOOT_SET_2);
                }
                break;

            case GOING_BACK_TO_SHOOT_SET_2:
                if (!follower.isBusy()) {
                    startShooting(set2Config);
                    setPathState(PathState.SHOOTING_SET_2);
                }
                break;

            case SHOOTING_SET_2:
                ShootingFunction.performShooting(launcher, intakeTransfer, shootTimer, activeConfig);

                if (shootTimer.getElapsedTimeSeconds() >= activeConfig.shootTimeSeconds) {
                    stopShooting();
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;
        }

        if (pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT && pathState != PathState.IDLE) {
            telemetry.addLine("WARNING: Path timeout exceeded!");
            setPathState(PathState.IDLE);
        }
    }

    private void startShooting(ShootingFunction.Configuration config) {
        activeConfig = config;
        launcher.setPower(config.ball1FlywheelPower);
        launcher.setHoodPosition(config.ball1HoodPosition);
        launcher.setSpinning(true);
        if (turretServo != null) {
            turretServo.setPosition(config.turretPosition);
        }
        intakeTransfer.transferUp();
        intakeTransfer.startIntake();
        shootTimer.resetTimer();
    }

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

    private void setPathState(PathState newState) {
        if (pathState != newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== Autonomous Status ===");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current Speed", String.format("%.0f%%", currentSpeed * 100));
        telemetry.addData("Path Time (s)", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("OpMode Time (s)", String.format("%.2f", opModeTimer.getElapsedTimeSeconds()));

        if (activeConfig != null) {
            telemetry.addLine();
            telemetry.addLine("=== Active Shooting Config ===");
            telemetry.addData("Ball 1 Power/Hood", String.format("%.2f / %.2f",
                    activeConfig.ball1FlywheelPower, activeConfig.ball1HoodPosition));
            telemetry.addData("Ball 2 Power/Hood", String.format("%.2f / %.2f",
                    activeConfig.ball2FlywheelPower, activeConfig.ball2HoodPosition));
            telemetry.addData("Ball 3 Power/Hood", String.format("%.2f / %.2f",
                    activeConfig.ball3FlywheelPower, activeConfig.ball3HoodPosition));
            telemetry.addData("Turret Position", activeConfig.turretPosition);

            if (pathState == PathState.PRELOAD_SHOOTING ||
                    pathState == PathState.SHOOTING_SET_1 ||
                    pathState == PathState.SHOOTING_SET_2) {
                String phase = ShootingFunction.getCurrentPhase(shootTimer, activeConfig);
                telemetry.addData("Phase", phase);
                telemetry.addData("Shoot Timer", String.format("%.2fs", shootTimer.getElapsedTimeSeconds()));
            }
        }

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
        if (launcher != null) {
            launcher.setSpinning(false);
        }
        if (intakeTransfer != null) {
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
        }
        follower.breakFollowing();
    }

    public static class Paths {
        public PathChain goingToNearestBalls;
        public PathChain gettingFirstBallSet1;
        public PathChain gettingSecondBallSet1;
        public PathChain gettingThirdBallSet1;
        public PathChain goingBackToShootSet1;
        public PathChain gettingNextSetOfBalls;
        public PathChain gettingFirstBallSet2;
        public PathChain gettingSecondBallSet2;
        public PathChain gettingThirdBallSet2;
        public PathChain goingBackToShootSet2;

        public Paths(Follower follower) {
            goingToNearestBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(24.143, 125.525), SPIKE1_CURVE_CONTROL, SPIKE1_APPROACH))
                    .setLinearHeadingInterpolation(HEADING_233, HEADING_180)
                    .build();

            gettingFirstBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_APPROACH, SPIKE1_BALL1))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingSecondBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL1, SPIKE1_BALL2))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingThirdBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL2, SPIKE1_BALL3))
                    .setTangentHeadingInterpolation()
                    .build();

            goingBackToShootSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL3, SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            gettingNextSetOfBalls = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, SPIKE2_APPROACH))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            gettingFirstBallSet2 = follower.pathBuilder()
                    .addPath(new BezierCurve(SPIKE2_APPROACH, SPIKE2_CURVE_CONTROL, SPIKE2_BALL1))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingSecondBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL1, SPIKE2_BALL2))
                    .setTangentHeadingInterpolation()
                    .build();

            gettingThirdBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL2, SPIKE2_BALL3))
                    .setTangentHeadingInterpolation()
                    .build();

            goingBackToShootSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL3, FINAL_SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();
        }
    }
}
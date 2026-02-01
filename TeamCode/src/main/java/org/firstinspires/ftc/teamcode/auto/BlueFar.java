package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.Pedro.Constants;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Combined Autonomous OpMode for Ball Collection Routine
 */
@Autonomous(name = "Blue Far", group = "Autonomous")
public class BlueFar extends OpMode {

    // ==================== CLASS FIELDS ====================
    private ShootingFunction.Configuration preloadConfig;
    private ShootingFunction.Configuration set1Config;
    private ShootingFunction.Configuration set2Config;
    private ShootingFunction.Configuration activeConfig;

    // ==================== PATH CONFIGURATION ====================
    private static final double PATH_SPEED = 0.6;
    private static final double PATH_TIMEOUT = 15.0;

    // ==================== POSE CONSTANTS ====================
    private static final double HEADING_180 = Math.toRadians(180);
    private static final Pose START_POSE = new Pose(57.845, 7.910, HEADING_180);
    private static final Pose SHOOT_POSE = new Pose(57.845, 7.910);
    private static final Pose SHOOT_POSE_OFFSET = new Pose(57.845, 7.910);
    private static final Pose SPIKE1_APPROACH = new Pose(41.172, 33.034);
    private static final Pose SPIKE1_BALL1 = new Pose(35.392, 32.899);
    private static final Pose SPIKE1_BALL2 = new Pose(29.358, 32.899);
    private static final Pose SPIKE1_BALL3 = new Pose(22.487, 32.899);
    private static final Pose SPIKE1_RETURN_START = new Pose(22.487, 30.899);
    private static final Pose SPIKE2_APPROACH = new Pose(41.2, 56.831);
    private static final Pose SPIKE2_BALL1 = new Pose(36.0, 56.730);
    private static final Pose SPIKE2_BALL2 = new Pose(31.568, 56.780);
    private static final Pose SPIKE2_BALL3 = new Pose(25.966, 57.008);
    private static final Pose SPIKE2_CURVE_CONTROL = new Pose(40.606, 42.856);

    private static final Pose LEAVE = new Pose (10, 10);

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
    private double initialTurretPosition = 0.5;  // Default center position, adjustable in init_loop

    public enum PathState {
        PRELOAD_SHOOT_SETUP, PRELOAD_SHOOTING,
        GOING_TO_NEAREST_BALLS, GETTING_FIRST_BALL_SET_1, GETTING_SECOND_BALL_SET_1, GETTING_THIRD_BALL_SET_1,
        GOING_BACK_TO_SHOOT_SET_1, SHOOTING_SET_1,
        GETTING_NEXT_SET_OF_BALLS, GETTING_FIRST_BALL_SET_2, GETTING_SECOND_BALL_SET_2, GETTING_THIRD_BALL_SET_2,
        GOING_BACK_TO_SHOOT_SET_2, SHOOTING_SET_2,
        LEAVE,
        IDLE
    }
    
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        // Initialize subsystems
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize turret servo and read current position
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            initialTurretPosition = turretServo.getPosition();
            telemetry.addLine("Turret Servo: OK");
            telemetry.addData("Initial Turret Position", "%.3f", initialTurretPosition);
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("Turret Servo: NOT FOUND");
        }

        // Load shooting configurations with turret position override
        preloadConfig = ShootingFunction.getConfigurationWithTurretOverride(
                ShootingFunction.AutonPath.BLUE_FAR,
                ShootingFunction.ShootingPosition.PRELOAD,
                initialTurretPosition
        );
        set1Config = ShootingFunction.getConfigurationWithTurretOverride(
                ShootingFunction.AutonPath.BLUE_FAR,
                ShootingFunction.ShootingPosition.SET_1,
                initialTurretPosition
        );
        set2Config = ShootingFunction.getConfigurationWithTurretOverride(
                ShootingFunction.AutonPath.BLUE_FAR,
                ShootingFunction.ShootingPosition.SET_2,
                initialTurretPosition
        );

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(PATH_SPEED);
        follower.activateAllPIDFs();
        paths = new Paths(follower);

        telemetry.addLine("Blue Far Combined Auto Initialized");
        telemetry.addData("Path Speed", "%.0f%%", PATH_SPEED * 100);
        telemetry.update();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
    }

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Allow turret position adjustment with DPAD LEFT/RIGHT during init
        if (turretServo != null) {
            // DPAD LEFT: Decrease turret position
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                initialTurretPosition -= 0.05;
                initialTurretPosition = Math.max(0.0, initialTurretPosition);
            }
            // DPAD RIGHT: Increase turret position
            else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                initialTurretPosition += 0.05;
                initialTurretPosition = Math.min(1.0, initialTurretPosition);
            }

            // Apply position to turret servo
            turretServo.setPosition(initialTurretPosition);

            // Update shooting configs with new turret position
            preloadConfig = ShootingFunction.getConfigurationWithTurretOverride(
                    ShootingFunction.AutonPath.BLUE_FAR,
                    ShootingFunction.ShootingPosition.PRELOAD,
                    initialTurretPosition
            );
            set1Config = ShootingFunction.getConfigurationWithTurretOverride(
                    ShootingFunction.AutonPath.BLUE_FAR,
                    ShootingFunction.ShootingPosition.SET_1,
                    initialTurretPosition
            );
            set2Config = ShootingFunction.getConfigurationWithTurretOverride(
                    ShootingFunction.AutonPath.BLUE_FAR,
                    ShootingFunction.ShootingPosition.SET_2,
                    initialTurretPosition
            );

            // Display current turret position
            telemetry.addLine("=== TURRET ADJUSTMENT ===");
            telemetry.addData("Turret Position", "%.3f", initialTurretPosition);
            telemetry.addLine("DPAD LEFT: Decrease (-0.01) | DPAD RIGHT: Increase (+0.01)");
            telemetry.update();
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
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);
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
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);
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
                    follower.followPath(paths.leave);
                    setPathState(PathState.LEAVE);
                }
                break;

            case LEAVE:
                if (!follower.isBusy()) {
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
        if (config.useRPMControl) {
            // Use RPM-based velocity control (for far zone matching teleop)
            launcher.setTargetRPM(config.ball1FlywheelPower);  // Value is RPM when useRPMControl=true
        } else {
            // Use direct power control (legacy)
            launcher.setPower(config.ball1FlywheelPower);
        }
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
        public PathChain leave;

        public Paths(Follower follower) {
            goingToNearestBalls = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, SPIKE1_APPROACH))
                    .setConstantHeadingInterpolation(HEADING_180)
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
                    .addPath(new BezierLine(SPIKE1_RETURN_START, SHOOT_POSE_OFFSET))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            gettingNextSetOfBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(SHOOT_POSE, SPIKE2_CURVE_CONTROL, SPIKE2_APPROACH))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            gettingFirstBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_APPROACH, SPIKE2_BALL1))
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
                    .addPath(new BezierLine(SPIKE2_BALL3, SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            leave = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, LEAVE))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();
        }
    }
}
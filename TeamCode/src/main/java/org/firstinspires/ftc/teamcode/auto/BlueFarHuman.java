package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
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
 * Blue Far Human Player Station - Preload only.
 * Starts at shoot pose (matches BlueFar), shoots the three preload balls, then moves to HUMAN_POSITION.
 */
@Autonomous(name = "Blue FAR Human", group = "Autonomous")
public class BlueFarHuman extends OpMode {

    // ==================== CLASS FIELDS ====================
    private ShootingFunction.Configuration preloadConfig;
    private ShootingFunction.Configuration activeConfig;

    // ==================== PATH CONFIGURATION ====================
    private static final double PATH_SPEED = 0.6;
    private static final double PATH_TIMEOUT = 15.0;

    // ==================== POSE CONSTANTS (match BlueFar) ====================
    private static final double HEADING_180 = Math.toRadians(180);
    private static final Pose START_POSE = new Pose(57.845, 7.910, HEADING_180);
    private static final Pose SHOOT_POSE = new Pose(57.845, 7.910);

    private static final Pose HUMAN_POSITION = new Pose(10, 10);  // Match BlueFar LEAVE


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
    private double initialTurretPosition = 0.5;

    public enum PathState {
        PRELOAD_SHOOT_SETUP, PRELOAD_SHOOTING,
        MOVING_TO_HUMAN_POSITION,
        IDLE
    }

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            initialTurretPosition = turretServo.getPosition();
            telemetry.addLine("Turret Servo: OK");
            telemetry.addData("Initial Turret Position", "%.3f", initialTurretPosition);
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("Turret Servo: NOT FOUND");
        }

        preloadConfig = ShootingFunction.getConfigurationWithTurretOverride(
                ShootingFunction.AutonPath.BLUE_FAR,
                ShootingFunction.ShootingPosition.PRELOAD,
                initialTurretPosition
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(PATH_SPEED);
        follower.activateAllPIDFs();
        paths = new Paths(follower);

        telemetry.addLine("Blue Far Human (Preload Only) Initialized");
        telemetry.addData("Path Speed", "%.0f%%", PATH_SPEED * 100);
        telemetry.update();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
    }

    @Override
    public void init_loop() {
        if (turretServo != null) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                initialTurretPosition -= 0.05;
                initialTurretPosition = Math.max(0.0, initialTurretPosition);
            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                initialTurretPosition += 0.05;
                initialTurretPosition = Math.min(1.0, initialTurretPosition);
            }

            turretServo.setPosition(initialTurretPosition);

            preloadConfig = ShootingFunction.getConfigurationWithTurretOverride(
                    ShootingFunction.AutonPath.BLUE_FAR,
                    ShootingFunction.ShootingPosition.PRELOAD,
                    initialTurretPosition
            );

            telemetry.addLine("=== TURRET ADJUSTMENT ===");
            telemetry.addData("Turret Position", "%.3f", initialTurretPosition);
            telemetry.addLine("DPAD LEFT: Decrease | DPAD RIGHT: Increase");
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

                if (shootTimer.getElapsedTimeSeconds() >= activeConfig.shootTimeSeconds) {
                    stopShooting();
                    follower.followPath(paths.movingToHumanPosition);
                    setPathState(PathState.MOVING_TO_HUMAN_POSITION);
                }
                break;

            case MOVING_TO_HUMAN_POSITION:
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
            launcher.setTargetRPM(config.ball1FlywheelPower);
        } else {
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
        telemetry.addLine("=== Blue Far Human (Preload Only) ===");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current Speed", String.format("%.0f%%", currentSpeed * 100));
        telemetry.addData("Path Time (s)", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("OpMode Time (s)", String.format("%.2f", opModeTimer.getElapsedTimeSeconds()));

        if (activeConfig != null) {
            telemetry.addLine();
            telemetry.addLine("=== Active Shooting Config ===");
            telemetry.addData("Turret Position", activeConfig.turretPosition);
            if (pathState == PathState.PRELOAD_SHOOTING) {
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
        public PathChain movingToHumanPosition;

        public Paths(Follower follower) {
            movingToHumanPosition = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, HUMAN_POSITION))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();
        }
    }
}

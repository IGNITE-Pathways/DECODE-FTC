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

@Autonomous(name = "Blue Far Arnavs", group = "Auto")
public class BlueFarArnavs extends OpMode {

    // ==================== SHOOTING CONSTANTS ====================
    // 10ft preset for preload shooting
    private static final double PRELOAD_FLYWHEEL_POWER = 0.67;
    private static final double PRELOAD_HOOD_POSITION = 0.70;
    private static final double PRELOAD_SHOOT_TIME_SECONDS = 6.0;

    // Turret locked position
    private static final double TURRET_LOCKED_POSITION = 0.47;

    // Path speed (45%)
    private static final double PATH_SPEED = 0.45;

    // ==================== ROBOT COMPONENTS ====================
    private Follower follower;
    private int pathState;

    // Robot components
    private IntakeTransfer intakeTransfer;
    private Launcher launcher;
    private Servo turretServo;

    // Timers
    private Timer pathTimer;
    private Timer opmodeTimer;
    private Timer shootTimer;

    // Paths
    private Paths paths;

    // Starting pose
    private final Pose startPose = new Pose(60.000, 12.364, Math.toRadians(180));

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        opmodeTimer.resetTimer();

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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(PATH_SPEED);  // Slow down to 45% speed

        // Build paths
        paths = new Paths(follower);

        telemetry.addLine("Blue Far Arnavs Auto Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep turret locked during init
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }
        telemetry.addLine("Blue Far Arnavs Auto ready!");
        telemetry.addData("Starting Pose", "X:%.2f Y:%.2f H:%.0f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Path Speed", "%.0f%%", PATH_SPEED * 100);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Update follower and launcher continuously
        follower.update();
        launcher.update();

        // Keep turret locked
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }

        // Run autonomous path state machine
        autonomousPathUpdate();

        // Telemetry for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", "%.2f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("Flywheel", launcher.isSpinning() ? "ON" : "OFF");
        telemetry.addData("Hood", "%.2f", launcher.getHoodPosition());
        telemetry.addData("Turret", "%.2f (locked)", TURRET_LOCKED_POSITION);
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));
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
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // PRELOAD SHOOT: 6 second wait with 10ft preset, shoot 3 preloaded balls
                // Set up shooter at 10ft preset
                launcher.setPower(PRELOAD_FLYWHEEL_POWER);
                launcher.setHoodPosition(PRELOAD_HOOD_POSITION);
                launcher.setSpinning(true);

                // Ramp up and start feeding balls
                intakeTransfer.transferUp();
                intakeTransfer.startIntake();

                shootTimer.resetTimer();
                setPathState(1);
                break;

            case 1:
                // Keep flywheel spinning during wait
                if (launcher.flyWheelMotor != null) {
                    launcher.flyWheelMotor.setPower(PRELOAD_FLYWHEEL_POWER);
                }
                if (launcher.flyWheelMotor2 != null) {
                    launcher.flyWheelMotor2.setPower(PRELOAD_FLYWHEEL_POWER);
                }
                launcher.setHoodPosition(PRELOAD_HOOD_POSITION);
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

                // Wait for 6 second preload shoot to complete
                if (elapsed >= PRELOAD_SHOOT_TIME_SECONDS) {
                    // Stop shooting
                    launcher.setSpinning(false);
                    if (launcher.flyWheelMotor != null) {
                        launcher.flyWheelMotor.setPower(0);
                    }
                    if (launcher.flyWheelMotor2 != null) {
                        launcher.flyWheelMotor2.setPower(0);
                    }
                    intakeTransfer.stopIntake();
                    intakeTransfer.transferDown();

                    // Start Path1 with intake running
                    intakeTransfer.startIntake();
                    follower.followPath(paths.Path1);
                    setPathState(2);
                }
                break;

            case 2:
                // Path1: Wait for completion, then start line2
                if (!follower.isBusy()) {
                    // Keep intake running, start line2
                    follower.followPath(paths.line2);
                    setPathState(3);
                }
                break;

            case 3:
                // line2: Wait for completion, then start line3
                if (!follower.isBusy()) {
                    // Keep intake running, start line3
                    follower.followPath(paths.line3);
                    setPathState(4);
                }
                break;

            case 4:
                // line3: Wait for completion, then start Path4
                if (!follower.isBusy()) {
                    // Stop intake for return path
                    intakeTransfer.stopIntake();
                    follower.followPath(paths.Path4);
                    setPathState(5);
                }
                break;

            case 5:
                // Path4: Wait for completion - DONE
                if (!follower.isBusy()) {
                    // All done
                    intakeTransfer.stopIntake();
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // ==================== PATHS ====================
    public static class Paths {
        public PathChain Path1;
        public PathChain line2;
        public PathChain line3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.000, 12.364),
                            new Pose(62.182, 34.545),
                            new Pose(37.273, 36.000)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(37.273, 36.000),
                            new Pose(27.273, 36.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            line3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(27.273, 36.000),
                            new Pose(9.091, 36.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.091, 36.000),
                            new Pose(59.818, 12.182)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}

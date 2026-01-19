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
    private static final double PRELOAD_FLYWHEEL_POWER = 0.75;
    private static final double PRELOAD_HOOD_POSITION = 0.60;
    private static final double PRELOAD_SHOOT_TIME_SECONDS = 7.0;

    // Intake cycling timing
    private static final double SPINUP_TIME_SECONDS = 1.5;
    private static final double INTAKE_ON_TIME_SECONDS = 0.7;
    private static final double INTAKE_OFF_TIME_SECONDS = 0.7;

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

    // Turret control flag
    private boolean turretEnabled = true;

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

        // Keep turret locked only when enabled (off during intaking)
        if (turretServo != null && turretEnabled) {
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
                // PRELOAD SHOOT: Start flywheel at 67% and ramp up first
                launcher.setPower(PRELOAD_FLYWHEEL_POWER);
                launcher.setHoodPosition(PRELOAD_HOOD_POSITION);
                launcher.setSpinning(true);

                // Ramp up first, intake will start after 1.5s delay
                intakeTransfer.transferUp();

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

                double elapsed = shootTimer.getElapsedTimeSeconds();

                // First 1.5 seconds - flywheel and ramp only, no intake
                if (elapsed < SPINUP_TIME_SECONDS) {
                    intakeTransfer.transferUp();
                    // Intake stays off during spinup
                }
                // After 1.5s - cycle intake: 0.7s on, 0.7s off, repeat 3 times
                else {
                    intakeTransfer.transferUp();
                    double cycleTime = elapsed - SPINUP_TIME_SECONDS;
                    double cycleLength = INTAKE_ON_TIME_SECONDS + INTAKE_OFF_TIME_SECONDS;
                    double positionInCycle = cycleTime % cycleLength;

                    if (positionInCycle < INTAKE_ON_TIME_SECONDS) {
                        // Intake ON phase
                        intakeTransfer.startIntake();
                    } else {
                        // Intake OFF phase (pause between balls)
                        intakeTransfer.stopIntake();
                    }
                }

                // Wait for 7 second preload shoot to complete
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

                    // Start Path1 with intake running - disable turret during intaking
                    turretEnabled = false;
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
                // Path4: Wait for completion, then start shooting
                if (!follower.isBusy()) {
                    // Re-enable turret for shooting
                    turretEnabled = true;

                    // Start shooting sequence - flywheel and ramp up first
                    launcher.setPower(PRELOAD_FLYWHEEL_POWER);
                    launcher.setHoodPosition(PRELOAD_HOOD_POSITION);
                    launcher.setSpinning(true);
                    intakeTransfer.transferUp();
                    shootTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                // Shooting after pickup: same 0.7s cycling pattern
                if (launcher.flyWheelMotor != null) {
                    launcher.flyWheelMotor.setPower(PRELOAD_FLYWHEEL_POWER);
                }
                if (launcher.flyWheelMotor2 != null) {
                    launcher.flyWheelMotor2.setPower(PRELOAD_FLYWHEEL_POWER);
                }
                launcher.setHoodPosition(PRELOAD_HOOD_POSITION);
                launcher.setSpinning(true);

                double elapsed2 = shootTimer.getElapsedTimeSeconds();

                // First 1.5 seconds - flywheel and ramp only, no intake
                if (elapsed2 < SPINUP_TIME_SECONDS) {
                    intakeTransfer.transferUp();
                    // Intake stays off during spinup
                }
                // After 1.5s - cycle intake: 0.7s on, 0.7s off, repeat 3 times
                else {
                    intakeTransfer.transferUp();
                    double cycleTime2 = elapsed2 - SPINUP_TIME_SECONDS;
                    double cycleLength2 = INTAKE_ON_TIME_SECONDS + INTAKE_OFF_TIME_SECONDS;
                    double positionInCycle2 = cycleTime2 % cycleLength2;

                    if (positionInCycle2 < INTAKE_ON_TIME_SECONDS) {
                        // Intake ON phase
                        intakeTransfer.startIntake();
                    } else {
                        // Intake OFF phase (pause between balls)
                        intakeTransfer.stopIntake();
                    }
                }

                // Wait for 7 second shoot to complete
                if (elapsed2 >= PRELOAD_SHOOT_TIME_SECONDS) {
                    // Stop shooting - all done
                    launcher.setSpinning(false);
                    if (launcher.flyWheelMotor != null) {
                        launcher.flyWheelMotor.setPower(0);
                    }
                    if (launcher.flyWheelMotor2 != null) {
                        launcher.flyWheelMotor2.setPower(0);
                    }
                    intakeTransfer.stopIntake();
                    intakeTransfer.transferDown();
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
                            new Pose(12.091, 36.000)
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
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
 * This autonomous routine performs the following sequence:
 * 1. Shoots 3 preloaded balls (6 second shooting sequence)
 * 2. Collects 3 balls from the first zone (spike mark 1)
 * 3. Returns to shoot position and shoots those 3 balls
 * 4. Collects 3 balls from the second zone (spike mark 2)
 * 5. Returns to shoot position and shoots those 3 balls
 * 6. Program ends after shooting the second set
 *
 * The robot uses dynamic speed control:
 * - Fast speed (80%) when approaching spike marks
 * - Slow speed (45%) during precise ball collection
 * - Full speed (100%) when returning to shooting position
 */
@Autonomous(name = "Red Far ACTUAL", group = "Autonomous")
public class RedFar extends OpMode {

    // ==================== SHOOTING CONSTANTS ====================
    // These constants define the 10ft shooting preset configuration
    private static final double FLYWHEEL_POWER = 1.0;  // Full power for flywheel motors
    private static final double HOOD_POSITION = 0.6;   // Hood servo position for 10ft shots
    private static final double SHOOT_TIME_SECONDS = 6.0;  // Total time allocated for each shooting sequence

    // Turret servo position - locked to prevent rotation during autonomous
    private static final double TURRET_LOCKED_POSITION = 0.3;

    // Path speed configuration - default speed for ball collection (45% power)
    private static final double PATH_SPEED = 0.45;

    // ==================== SHOOTING SEQUENCE TIMING ====================
    // Configure the precise timing for the 3-ball shooting sequence
    // All times are in seconds - TUNE THESE VALUES to optimize shooting performance

    // ===== SPINUP PHASE =====
    // Initial flywheel spin-up period
    private static final double SPINUP_TIME = 2.0;           // How long to wait for flywheel to reach full speed
    private static final boolean SPINUP_INTAKE_ON = false;   // Should intake be ON during spinup? (false = OFF)
    private static final boolean SPINUP_EJECT_ON = false;    // Should eject be ON during spinup? (false = normal)

    // ===== BALL 1 FEED PHASE =====
    private static final double BALL1_FEED_TIME = 0.15;      // How long ramp is UP to feed ball 1
    private static final boolean BALL1_FEED_INTAKE_ON = true;  // Should intake be ON during ball 1 feed?
    private static final boolean BALL1_FEED_EJECT_ON = false;  // Should eject be ON during ball 1 feed?

    // ===== BALL 1 RECOVERY PHASE =====
    private static final double BALL1_RECOVERY_TIME = 0.5;   // Wait time after ball 1 shoots (ramp down)
    private static final boolean BALL1_RECOVERY_INTAKE_ON = true;  // Should intake be ON during ball 1 recovery?
    private static final boolean BALL1_RECOVERY_EJECT_ON = false;  // Should eject be ON during ball 1 recovery?

    // ===== BALL 2 FEED PHASE =====
    private static final double BALL2_FEED_TIME = 0.3;       // How long ramp is UP to feed ball 2
    private static final boolean BALL2_FEED_INTAKE_ON = true;  // Should intake be ON during ball 2 feed?
    private static final boolean BALL2_FEED_EJECT_ON = false;  // Should eject be ON during ball 2 feed?

    // ===== BALL 2 RECOVERY PHASE =====
    private static final double BALL2_RECOVERY_TIME = 0.5;   // Wait time after ball 2 shoots (ramp down)
    private static final boolean BALL2_RECOVERY_INTAKE_ON = true;  // Should intake be ON during ball 2 recovery?
    private static final boolean BALL2_RECOVERY_EJECT_ON = false;  // Should eject be ON during ball 2 recovery?

    // ===== BALL 3 FEED PHASE =====
    private static final double BALL3_FEED_TIME = 0.3;       // How long ramp is UP to feed ball 3
    private static final boolean BALL3_FEED_INTAKE_ON = true;  // Should intake be ON during ball 3 feed?
    private static final boolean BALL3_FEED_EJECT_ON = false;  // Should eject be ON during ball 3 feed?

    // ===== FINISH PHASE =====
    // No recovery time needed after ball 3 since sequence ends
    private static final boolean FINISH_INTAKE_ON = false;   // Should intake be ON after all balls shot?
    private static final boolean FINISH_EJECT_ON = false;    // Should eject be ON after all balls shot?

    // CALCULATED TIME MARKERS (DO NOT MODIFY - these are auto-calculated from above values)
    // These mark the exact timestamps when each action should occur
    private static final double BALL1_START = SPINUP_TIME;
    private static final double BALL1_END = BALL1_START + BALL1_FEED_TIME;
    private static final double BALL2_START = BALL1_END + BALL1_RECOVERY_TIME;
    private static final double BALL2_END = BALL2_START + BALL2_FEED_TIME;
    private static final double BALL3_START = BALL2_END + BALL2_RECOVERY_TIME;
    private static final double BALL3_END = BALL3_START + BALL3_FEED_TIME;

    // Safety timeout to prevent infinite loops if paths get stuck (15 seconds per path)
    private static final double PATH_TIMEOUT = 15.0;

    // ==================== POSE CONSTANTS ====================
    // All positions are in inches, heading is in radians
    // The field uses a coordinate system where (0,0) is a corner of the field

    // Common heading for all positions - facing 0 degrees
    private static final double HEADING_0 = Math.toRadians(0);

    // Starting position - where the robot begins autonomous
    private static final Pose START_POSE = new Pose(84.652, 9.071, HEADING_0);

    // Shooting position - where robot returns to shoot collected balls
    private static final Pose SHOOT_POSE = new Pose(84.652, 9.071, HEADING_0);

    // First spike mark positions (Set 1) - closest to starting position
    private static final Pose SPIKE1_APPROACH = new Pose(101.559, 35.453);  // Approach point before collecting
    private static final Pose SPIKE1_BALL1 = new Pose(107.779, 35.511);     // First ball location
    private static final Pose SPIKE1_BALL2 = new Pose(113.745, 35.511);     // Second ball location
    private static final Pose SPIKE1_BALL3 = new Pose(119.649, 35.511);     // Third ball location

    // Second spike mark positions (Set 2) - middle spike mark
    private static final Pose SPIKE2_APPROACH = new Pose(101.300, 59.831);  // Approach point
    private static final Pose SPIKE2_BALL1 = new Pose(108.322, 59.923);     // First ball
    private static final Pose SPIKE2_BALL2 = new Pose(114.068, 59.780);     // Second ball
    private static final Pose SPIKE2_BALL3 = new Pose(119.792, 59.815);     // Third ball

    // Final shooting position
    private static final Pose FINAL_SHOOT_POSE = new Pose(84.581, 9.000);

    // Curve control point for smooth path to spike 2 (creates curved approach instead of straight line)
    private static final Pose SPIKE2_CURVE_CONTROL = new Pose(96.735, 54.372);

    // ==================== ROBOT COMPONENTS ====================
    // Pedro Pathing follower - handles path following and autonomous navigation
    private Follower follower;
    // Path definitions object - contains all pre-built paths
    private Paths paths;

    // Robot subsystems
    private IntakeTransfer intakeTransfer;  // Controls intake motors and transfer ramp
    private Launcher launcher;              // Controls flywheel and hood for shooting
    private Servo turretServo;              // Turret servo (locked during autonomous)

    // ==================== STATE MANAGEMENT ====================
    // Current state in the autonomous sequence
    private PathState pathState = PathState.PRELOAD_SHOOT_SETUP;

    // Timers for various operations
    private Timer pathTimer;     // Tracks time in current path state (for timeout detection)
    private Timer opModeTimer;   // Tracks total autonomous runtime
    private Timer shootTimer;    // Tracks shooting sequence timing
    private Timer ejectTimer;    // Timer for eject/reintake sequence (currently unused)
    private Timer rampTimer;     // Timer for ramp down/up sequence (currently unused)

    // Flags for shooting sequence control (currently unused but kept for future modifications)
    private boolean ejectReintakeDone = false;      // Track if eject/reintake has been done
    private boolean ejectReintakeStarted = false;   // Track if eject/reintake sequence has started
    private boolean rampSequenceDone = false;       // Track if ramp down/up sequence has been done
    private boolean rampSequenceStarted = false;    // Track if ramp sequence has started
    private boolean rampDownCalled = false;         // Track if ramp down has been called

    // Current robot speed - tracked for telemetry display
    private double currentSpeed = PATH_SPEED;

    /**
     * Path State Enumeration
     * Defines all possible states in the autonomous routine
     * The state machine progresses through these states in order
     */
    public enum PathState {
        // ========== PRELOAD SHOOTING ==========
        PRELOAD_SHOOT_SETUP,   // Set up shooter and prepare for preload shooting
        PRELOAD_SHOOTING,      // Execute 6-second preload shooting sequence

        // ========== FIRST BALL SET (Starting Zone) ==========
        GOING_TO_NEAREST_BALLS,     // Fast approach to first spike mark
        GETTING_FIRST_BALL_SET_1,   // Collect first ball from spike 1
        GETTING_SECOND_BALL_SET_1,  // Collect second ball from spike 1
        GETTING_THIRD_BALL_SET_1,   // Collect third ball from spike 1
        GOING_BACK_TO_SHOOT_SET_1,  // Fast return to shooting position
        SHOOTING_SET_1,             // Shoot the 3 collected balls

        // ========== SECOND BALL SET ==========
        GETTING_NEXT_SET_OF_BALLS,  // Fast approach to second spike mark
        GETTING_FIRST_BALL_SET_2,   // Collect first ball from spike 2
        GETTING_SECOND_BALL_SET_2,  // Collect second ball from spike 2
        GETTING_THIRD_BALL_SET_2,   // Collect third ball from spike 2
        GOING_BACK_TO_SHOOT_SET_2,  // Fast return to shooting position
        SHOOTING_SET_2,             // Shoot the 3 collected balls

        IDLE  // Autonomous complete - all systems off
    }

    /**
     * Initialization method - called once when autonomous is selected
     * Sets up all robot components and prepares for autonomous execution
     */
    @Override
    public void init() {
        // Initialize all timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();
        ejectTimer = new Timer();
        rampTimer = new Timer();

        // Initialize intake/transfer subsystem
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        // Initialize launcher subsystem (flywheel + hood)
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize turret servo and lock it in position
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            turretServo.setPosition(TURRET_LOCKED_POSITION);
            telemetry.addLine("Turret Servo: OK");
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("Turret Servo: NOT FOUND");
        }

        // Initialize Pedro Pathing Follower for autonomous navigation
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);  // Set initial robot position
        follower.setMaxPower(PATH_SPEED);      // Set default path following speed

        // Activate all PID controllers so tuned PIDF constants are used
        follower.activateAllPIDFs();

        // Create all path objects
        paths = new Paths(follower);

        // Display initialization status
        telemetry.addLine("Red Far Combined Auto Initialized");
        telemetry.addData("Path Speed", "%.0f%%", PATH_SPEED * 100);
        telemetry.addLine();
        telemetry.addLine("=== Shooting Timing ===");
        telemetry.addData("Spinup", "%.2fs (I:%s E:%s)", SPINUP_TIME,
                SPINUP_INTAKE_ON ? "ON" : "OFF", SPINUP_EJECT_ON ? "ON" : "OFF");
        telemetry.addData("Ball 1 Feed", "%.2fs (I:%s E:%s)", BALL1_FEED_TIME,
                BALL1_FEED_INTAKE_ON ? "ON" : "OFF", BALL1_FEED_EJECT_ON ? "ON" : "OFF");
        telemetry.addData("Ball 1 Recovery", "%.2fs (I:%s E:%s)", BALL1_RECOVERY_TIME,
                BALL1_RECOVERY_INTAKE_ON ? "ON" : "OFF", BALL1_RECOVERY_EJECT_ON ? "ON" : "OFF");
        telemetry.addData("Ball 2 Feed", "%.2fs (I:%s E:%s)", BALL2_FEED_TIME,
                BALL2_FEED_INTAKE_ON ? "ON" : "OFF", BALL2_FEED_EJECT_ON ? "ON" : "OFF");
        telemetry.addData("Ball 2 Recovery", "%.2fs (I:%s E:%s)", BALL2_RECOVERY_TIME,
                BALL2_RECOVERY_INTAKE_ON ? "ON" : "OFF", BALL2_RECOVERY_EJECT_ON ? "ON" : "OFF");
        telemetry.addData("Ball 3 Feed", "%.2fs (I:%s E:%s)", BALL3_FEED_TIME,
                BALL3_FEED_INTAKE_ON ? "ON" : "OFF", BALL3_FEED_EJECT_ON ? "ON" : "OFF");
        telemetry.addData("Total Sequence", "%.2fs", BALL3_END);
        telemetry.update();
    }

    /**
     * Init loop - called repeatedly after init() until START is pressed
     * Keeps turret locked during initialization phase
     */
    @Override
    public void init_loop() {
        // Keep turret locked during init
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }
    }

    /**
     * Start method - called once when START button is pressed
     * Resets all timers and begins autonomous sequence
     */
    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        shootTimer.resetTimer();
        pathState = PathState.PRELOAD_SHOOT_SETUP;  // Begin with preload shooting
    }

    /**
     * Main loop - called repeatedly during autonomous execution
     * Updates all systems and progresses through state machine
     */
    @Override
    public void loop() {
        // CRITICAL: Update follower and launcher every loop iteration
        // Follower update processes path following and motor control
        follower.update();
        // Launcher update handles flywheel ramping and hood positioning
        launcher.update();

        // Keep turret locked throughout autonomous
        if (turretServo != null) {
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        }

        // Execute current path state logic
        autonomousPathUpdate();

        // Update driver station telemetry
        updateTelemetry();
    }

    /**
     * State machine for autonomous path execution
     * This method contains the core logic for transitioning between states
     * and executing actions for each state
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            // ========== PRELOAD SHOOTING ==========
            case PRELOAD_SHOOT_SETUP:
                // Configure shooter for 10ft preset
                launcher.setPower(FLYWHEEL_POWER);
                launcher.setHoodPosition(HOOD_POSITION);
                launcher.setSpinning(true);  // Start flywheel spinning

                // Raise transfer ramp and start intake to feed balls
                intakeTransfer.transferUp();
                intakeTransfer.startIntake();

                // Reset shooting timer and flags
                shootTimer.resetTimer();
                ejectReintakeDone = false;
                ejectReintakeStarted = false;
                rampSequenceDone = false;
                rampSequenceStarted = false;
                rampDownCalled = false;

                // Transition to shooting state
                setPathState(PathState.PRELOAD_SHOOTING);
                break;

            case PRELOAD_SHOOTING:
                // Execute timed shooting sequence for 3 preloaded balls
                performShooting();

                // Wait for full shooting duration (6 seconds)
                double elapsed = shootTimer.getElapsedTimeSeconds();
                if (elapsed >= SHOOT_TIME_SECONDS) {
                    // Shooting complete - turn off shooter systems
                    stopShooting();

                    // Start approaching first spike mark with FAST SPEED (80%)
                    currentSpeed = 0.8;
                    follower.setMaxPower(0.8);

                    // Start intake to collect balls during approach
                    intakeTransfer.startIntake();

                    // Begin following path to first spike mark
                    follower.followPath(paths.goingToNearestBalls);
                    setPathState(PathState.GOING_TO_NEAREST_BALLS);
                }
                break;

            // ========== FIRST BALL SET ==========
            case GOING_TO_NEAREST_BALLS:
                // Keep intake running while approaching spike mark
                if (!follower.isBusy()) {
                    // Path complete - now SLOW DOWN for precise ball collection (45%)
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);

                    // Begin collecting first ball
                    follower.followPath(paths.gettingFirstBallSet1);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_1);
                }
                break;

            case GETTING_FIRST_BALL_SET_1:
                // Keep intake running to collect ball
                if (!follower.isBusy()) {
                    // First ball collected - move to second ball
                    follower.followPath(paths.gettingSecondBallSet1);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_1);
                }
                break;

            case GETTING_SECOND_BALL_SET_1:
                // Keep intake running to collect ball
                if (!follower.isBusy()) {
                    // Second ball collected - move to third ball
                    follower.followPath(paths.gettingThirdBallSet1);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_1);
                }
                break;

            case GETTING_THIRD_BALL_SET_1:
                // Keep intake running to collect ball
                if (!follower.isBusy()) {
                    // All 3 balls collected - stop intake and return to shooting position
                    intakeTransfer.stopIntake();

                    // GO MAX SPEED (100%) for fast return
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);

                    // Follow return path to shooting position
                    follower.followPath(paths.goingBackToShootSet1);
                    setPathState(PathState.GOING_BACK_TO_SHOOT_SET_1);
                }
                break;

            case GOING_BACK_TO_SHOOT_SET_1:
                // Wait for robot to reach shooting position
                if (!follower.isBusy()) {
                    // At shooting position - start shooting sequence
                    startShooting();
                    setPathState(PathState.SHOOTING_SET_1);
                }
                break;

            case SHOOTING_SET_1:
                // Execute timed shooting sequence
                performShooting();

                // Wait for shooting to complete
                if (shootTimer.getElapsedTimeSeconds() >= SHOOT_TIME_SECONDS) {
                    // First set shot - turn off shooter
                    stopShooting();

                    // Start approaching second spike mark with FAST SPEED (80%)
                    currentSpeed = 0.8;
                    follower.setMaxPower(0.8);

                    // Start intake for next collection
                    intakeTransfer.startIntake();

                    // Follow curved path to second spike mark
                    follower.followPath(paths.gettingNextSetOfBalls);
                    setPathState(PathState.GETTING_NEXT_SET_OF_BALLS);
                }
                break;

            // ========== SECOND BALL SET ==========
            case GETTING_NEXT_SET_OF_BALLS:
                // Keep intake running while approaching spike mark
                if (!follower.isBusy()) {
                    // Path complete - now SLOW DOWN for precise ball collection (45%)
                    currentSpeed = PATH_SPEED;
                    follower.setMaxPower(PATH_SPEED);

                    // Begin collecting first ball from spike 2
                    follower.followPath(paths.gettingFirstBallSet2);
                    setPathState(PathState.GETTING_FIRST_BALL_SET_2);
                }
                break;

            case GETTING_FIRST_BALL_SET_2:
                // Keep intake running to collect ball
                if (!follower.isBusy()) {
                    // First ball collected - move to second ball
                    follower.followPath(paths.gettingSecondBallSet2);
                    setPathState(PathState.GETTING_SECOND_BALL_SET_2);
                }
                break;

            case GETTING_SECOND_BALL_SET_2:
                // Keep intake running to collect ball
                if (!follower.isBusy()) {
                    // Second ball collected - move to third ball
                    follower.followPath(paths.gettingThirdBallSet2);
                    setPathState(PathState.GETTING_THIRD_BALL_SET_2);
                }
                break;

            case GETTING_THIRD_BALL_SET_2:
                // Keep intake running to collect ball
                if (!follower.isBusy()) {
                    // All 3 balls collected - stop intake and return to shooting position
                    intakeTransfer.stopIntake();

                    // GO MAX SPEED (100%) for fast return
                    currentSpeed = 1.0;
                    follower.setMaxPower(1.0);

                    // Follow return path to shooting position
                    follower.followPath(paths.goingBackToShootSet2);
                    setPathState(PathState.GOING_BACK_TO_SHOOT_SET_2);
                }
                break;

            case GOING_BACK_TO_SHOOT_SET_2:
                // Wait for robot to reach shooting position
                if (!follower.isBusy()) {
                    // At shooting position - start shooting sequence
                    startShooting();
                    setPathState(PathState.SHOOTING_SET_2);
                }
                break;

            case SHOOTING_SET_2:
                // Execute timed shooting sequence
                performShooting();

                // Wait for shooting to complete
                if (shootTimer.getElapsedTimeSeconds() >= SHOOT_TIME_SECONDS) {
                    // Second set shot - turn off all systems and go idle
                    stopShooting();
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                // Autonomous complete - all systems off
                break;
        }

        // Safety timeout - prevent infinite loops if a path gets stuck
        if (pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT && pathState != PathState.IDLE) {
            telemetry.addLine("WARNING: Path timeout exceeded!");
            setPathState(PathState.IDLE);
        }
    }

    // [REST OF THE METHODS ARE IDENTICAL TO BlueFarActual - startShooting(), performEjectReintake(),
    // performRampDownUp(), performShooting(), stopShooting(), setPathState(), updateTelemetry(), stop()]
    // I'll include them but they're exact copies from BlueFarActual

    private void startShooting() {
        launcher.setPower(FLYWHEEL_POWER);
        launcher.setHoodPosition(HOOD_POSITION);
        launcher.setSpinning(true);
        intakeTransfer.transferUp();
        intakeTransfer.startIntake();
        shootTimer.resetTimer();
        ejectReintakeDone = false;
        ejectReintakeStarted = false;
        rampSequenceDone = false;
        rampSequenceStarted = false;
        rampDownCalled = false;
    }

    private void performEjectReintake() {
        intakeTransfer.transferUp();
        double elapsed = ejectTimer.getElapsedTimeSeconds();
        if (elapsed < 0.1) {
            intakeTransfer.startEject(1.0);
        } else {
            intakeTransfer.startIntake();
            ejectReintakeDone = true;
        }
    }

    private void performRampDownUp() {
        double elapsed = rampTimer.getElapsedTimeSeconds();
        if (!rampDownCalled) {
            intakeTransfer.transferDown();
            rampDownCalled = true;
        }
        if (elapsed < 0.2) {
            intakeTransfer.startEject(1.0);
        } else if (elapsed < 0.4) {
            intakeTransfer.startIntake();
        } else {
            intakeTransfer.transferUp();
            rampSequenceDone = true;
        }
    }

    private void performShooting() {
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(FLYWHEEL_POWER);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(FLYWHEEL_POWER);
        }
        launcher.setHoodPosition(HOOD_POSITION);
        launcher.setSpinning(true);

        double elapsed = shootTimer.getElapsedTimeSeconds();

        if (elapsed < SPINUP_TIME) {
            intakeTransfer.transferDown();
            if (SPINUP_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (SPINUP_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
        else if (elapsed >= BALL1_START && elapsed < BALL1_END) {
            intakeTransfer.transferUp();
            if (BALL1_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL1_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
        else if (elapsed >= BALL1_END && elapsed < BALL2_START) {
            intakeTransfer.transferDown();
            if (BALL1_RECOVERY_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL1_RECOVERY_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
        else if (elapsed >= BALL2_START && elapsed < BALL2_END) {
            intakeTransfer.transferUp();
            if (BALL2_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL2_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
        else if (elapsed >= BALL2_END && elapsed < BALL3_START) {
            intakeTransfer.transferDown();
            if (BALL2_RECOVERY_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL2_RECOVERY_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
        else if (elapsed >= BALL3_START && elapsed < BALL3_END) {
            intakeTransfer.transferUp();
            if (BALL3_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL3_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
        else if (elapsed >= BALL3_END) {
            intakeTransfer.transferDown();
            if (FINISH_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (FINISH_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }
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

        telemetry.addLine();
        telemetry.addLine("=== Shooter Status ===");
        telemetry.addData("Flywheel", launcher.isSpinning() ? "ON" : "OFF");
        telemetry.addData("Hood Position", String.format("%.2f", launcher.getHoodPosition()));
        telemetry.addData("Turret", String.format("%.2f (locked)", TURRET_LOCKED_POSITION));

        if (pathState == PathState.PRELOAD_SHOOTING ||
                pathState == PathState.SHOOTING_SET_1 ||
                pathState == PathState.SHOOTING_SET_2) {
            double elapsed = shootTimer.getElapsedTimeSeconds();
            telemetry.addData("Shoot Timer", String.format("%.2fs", elapsed));

            if (elapsed < SPINUP_TIME) {
                telemetry.addData("Phase", "SPINUP");
            } else if (elapsed < BALL1_END) {
                telemetry.addData("Phase", "BALL 1 FEED");
            } else if (elapsed < BALL2_START) {
                telemetry.addData("Phase", "BALL 1 RECOVERY");
            } else if (elapsed < BALL2_END) {
                telemetry.addData("Phase", "BALL 2 FEED");
            } else if (elapsed < BALL3_START) {
                telemetry.addData("Phase", "BALL 2 RECOVERY");
            } else if (elapsed < BALL3_END) {
                telemetry.addData("Phase", "BALL 3 FEED");
            } else {
                telemetry.addData("Phase", "FINISH");
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

    /**
     * ========================================
     * PATH DEFINITIONS - Inner Class
     * ========================================
     *
     * All paths use extracted Pose constants for maintainability
     * This inner class builds all the paths used during autonomous
     */
    public static class Paths {
        // ========== FIRST BALL SET PATHS ==========
        public PathChain goingToNearestBalls;     // Fast approach to spike 1
        public PathChain gettingFirstBallSet1;    // Collect ball 1 from spike 1
        public PathChain gettingSecondBallSet1;   // Collect ball 2 from spike 1
        public PathChain gettingThirdBallSet1;    // Collect ball 3 from spike 1
        public PathChain goingBackToShootSet1;    // Return to shooting position

        // ========== SECOND BALL SET PATHS ==========
        public PathChain gettingNextSetOfBalls;   // Curved approach to spike 2
        public PathChain gettingFirstBallSet2;    // Collect ball 1 from spike 2
        public PathChain gettingSecondBallSet2;   // Collect ball 2 from spike 2
        public PathChain gettingThirdBallSet2;    // Collect ball 3 from spike 2
        public PathChain goingBackToShootSet2;    // Return to shooting position

        /**
         * Constructor - Build all paths using pose constants
         * Paths are built using either BezierLine (straight) or BezierCurve (curved)
         * Heading interpolation determines how the robot rotates during the path:
         * - setConstantHeadingInterpolation: Robot maintains constant heading
         * - setTangentHeadingInterpolation: Robot heading follows path tangent
         */
        public Paths(Follower follower) {
            // ========== FIRST BALL SET PATHS ==========

            // Path 1: Straight approach from starting position to spike 1 approach point
            goingToNearestBalls = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SPIKE1_APPROACH))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 2: Slow precise movement to first ball
            gettingFirstBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_APPROACH, SPIKE1_BALL1))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 3: Slow precise movement to second ball
            gettingSecondBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL1, SPIKE1_BALL2))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 4: Slow precise movement to third ball
            gettingThirdBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL2, SPIKE1_BALL3))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 5: Fast return to shooting position
            goingBackToShootSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL3, SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // ========== SECOND BALL SET PATHS ==========

            // Path 6: Curved approach to spike 2
            gettingNextSetOfBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            SPIKE2_CURVE_CONTROL,
                            SPIKE2_APPROACH
                    ))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 7: Slow precise movement to first ball of spike 2
            gettingFirstBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_APPROACH, SPIKE2_BALL1))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 8: Slow precise movement to second ball of spike 2
            gettingSecondBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL1, SPIKE2_BALL2))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 9: Slow precise movement to third ball of spike 2
            gettingThirdBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL2, SPIKE2_BALL3))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();

            // Path 10: Fast return to final shooting position
            goingBackToShootSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL3, FINAL_SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_0)
                    .build();
        }
    }
}
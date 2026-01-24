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




// NOTES FOR AUTON POSITIONS

// IN PRELOAD MAKE FLYWHEEL POWER 0.58


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
@Autonomous(name = "Blue Near ACTUAL", group = "Autonomous")
public class BlueNear extends OpMode {

    // ==================== SHOOTING CONSTANTS ====================
// PRELOAD SHOOTING CONFIGURATION
    private static final double PRELOAD_FLYWHEEL_POWER = 0.6;  // Per note: use 0.58 for preload
    private static final double PRELOAD_HOOD_POSITION = 0.35;
    private static final double PRELOAD_TURRET_POSITION = 0.75;

    // FIRST SPIKE MARK SHOOTING CONFIGURATION
    private static final double SET1_FLYWHEEL_POWER = 0.6;
    private static final double SET1_HOOD_POSITION = 0.7;
    private static final double SET1_TURRET_POSITION = 0.6;

    // SECOND SPIKE MARK SHOOTING CONFIGURATION
    private static final double SET2_FLYWHEEL_POWER = 0.55;
    private static final double SET2_HOOD_POSITION = 0.6;
    private static final double SET2_TURRET_POSITION = 0.6;

    // Shared shooting timing constants (same for all sets)
    private static final double SHOOT_TIME_SECONDS = 6.0;

    // Active shooting configuration (will be updated based on current set)
    private double activeFlywheelPower = PRELOAD_FLYWHEEL_POWER;
    private double activeHoodPosition = PRELOAD_HOOD_POSITION;
    private double activeTurretPosition = PRELOAD_TURRET_POSITION;

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

    // Common heading for all positions - facing 233 degrees
    private static final double HEADING_233 = Math.toRadians(233);
    private static final double HEADING_180 = Math.toRadians(180);

    // Starting position - where the robot begins autonomous
    private static final Pose START_POSE = new Pose(24.143, 125.525, HEADING_233);

    // Shooting position - where robot returns to shoot collected balls
    private static final Pose SHOOT_POSE = new Pose(48.045, 96.350);

    // First spike mark positions (Set 1) - closest to starting position
    private static final Pose SPIKE1_APPROACH = new Pose(41.794, 84.215);  // Approach point before collecting
    private static final Pose SPIKE1_BALL1 = new Pose(36.099, 84.072);     // First ball location
    private static final Pose SPIKE1_BALL2 = new Pose(30.556, 84.099);     // Second ball location
    private static final Pose SPIKE1_BALL3 = new Pose(25.408, 84.117);     // Third ball location

    // Second spike mark positions (Set 2) - middle spike mark
    private static final Pose SPIKE2_APPROACH = new Pose(41.314, 60.444);  // Approach point
    private static final Pose SPIKE2_BALL1 = new Pose(36.265, 60.283);     // First ball
    private static final Pose SPIKE2_BALL2 = new Pose(31.090, 60.157);     // Second ball
    private static final Pose SPIKE2_BALL3 = new Pose(25.906, 60.184);     // Third ball

    // Final shooting position
    private static final Pose FINAL_SHOOT_POSE = new Pose(60.673, 81.327);

    // Curve control point for path from start to spike 1
    private static final Pose SPIKE1_CURVE_CONTROL = new Pose(48.215, 96.000);

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
        telemetry.addLine("Blue Near Combined Auto Initialized");
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
                setShootingConfiguration(PathState.PRELOAD_SHOOTING);
                launcher.setPower(activeFlywheelPower);
                launcher.setHoodPosition(activeHoodPosition);

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
                if (!follower.isBusy()) {
                    startShooting(PathState.SHOOTING_SET_1);
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

                    // Follow path to second spike mark
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
                if (!follower.isBusy()) {
                    startShooting(PathState.SHOOTING_SET_2);
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
    /**
     * Set shooting configuration based on which set of balls is being shot
     */
    private void setShootingConfiguration(PathState state) {
        if (state == PathState.PRELOAD_SHOOTING) {
            activeFlywheelPower = PRELOAD_FLYWHEEL_POWER;
            activeHoodPosition = PRELOAD_HOOD_POSITION;
            activeTurretPosition = PRELOAD_TURRET_POSITION;
        } else if (state == PathState.SHOOTING_SET_1) {
            activeFlywheelPower = SET1_FLYWHEEL_POWER;
            activeHoodPosition = SET1_HOOD_POSITION;
            activeTurretPosition = SET1_TURRET_POSITION;
        } else if (state == PathState.SHOOTING_SET_2) {
            activeFlywheelPower = SET2_FLYWHEEL_POWER;
            activeHoodPosition = SET2_HOOD_POSITION;
            activeTurretPosition = SET2_TURRET_POSITION;
        }
    }


    /**
     * Start shooting sequence
     * Sets up shooter configuration, raises transfer ramp, and starts intake
     */
    private void startShooting(PathState shootingState) {
        // Set the appropriate shooting configuration
        setShootingConfiguration(shootingState);

        // Configure shooter with active configuration
        launcher.setPower(activeFlywheelPower);
        launcher.setHoodPosition(activeHoodPosition);
        launcher.setSpinning(true);

        // Set turret to active position
        if (turretServo != null) {
            turretServo.setPosition(activeTurretPosition);
        }

        // Raise transfer ramp and start intake to feed balls
        intakeTransfer.transferUp();
        intakeTransfer.startIntake();

        // Reset timer and flags for new shooting sequence
        shootTimer.resetTimer();
        ejectReintakeDone = false;
        ejectReintakeStarted = false;
        rampSequenceDone = false;
        rampSequenceStarted = false;
        rampDownCalled = false;
    }

    /**
     * Eject ball for 0.2 seconds then reintake
     * This helps clear any stuck balls and ensures the third ball feeds properly
     * The ramp stays UP during this sequence to allow ball transfer
     *
     * NOTE: This method is currently unused but kept for future modifications
     */
    private void performEjectReintake() {
        // Keep ramp UP during eject/reintake sequence
        intakeTransfer.transferUp();

        double elapsed = ejectTimer.getElapsedTimeSeconds();

        if (elapsed < 0.1) {
            // Eject phase - run motor in reverse (0.1 seconds)
            intakeTransfer.startEject(1.0);  // Full power eject
        } else {
            // Reintake phase - switch back to normal intake
            intakeTransfer.startIntake();
            ejectReintakeDone = true;  // Mark as complete
        }
    }

    /**
     * Move ramp down, eject, reintake, then ramp up for third ball shooting
     * Sequence: Ramp down → Eject (0.2s) → Reintake (0.2s) → Ramp up
     * This helps ensure the third ball feeds properly by cycling the ramp and clearing any stuck balls
     *
     * NOTE: This method is currently unused but kept for future modifications
     */
    private void performRampDownUp() {
        double elapsed = rampTimer.getElapsedTimeSeconds();

        // Phase 1: Ramp down (only once at the start)
        if (!rampDownCalled) {
            intakeTransfer.transferDown();
            rampDownCalled = true;
        }

        if (elapsed < 0.2) {
            // Phase 2: Eject for 0.2 seconds
            intakeTransfer.startEject(1.0);  // Full power eject
        } else if (elapsed < 0.4) {
            // Phase 3: Reintake for 0.2 seconds
            intakeTransfer.startIntake();
        } else {
            // Phase 4: Ramp up at 0.4 second mark
            intakeTransfer.transferUp();
            rampSequenceDone = true;  // Mark as complete
        }
    }

    /**
     * ========================================================================
     * PERFORM SHOOTING - MAIN SHOOTING SEQUENCE
     * ========================================================================
     *
     * This method executes the timed 3-ball shooting sequence using the timing
     * constants defined at the top of the class.
     *
     * TO TUNE THE SHOOTING SEQUENCE:
     * 1. Modify the timing constants at the top of this file:
     *    TIME CONSTANTS:
     *      - SPINUP_TIME: How long to wait for flywheel to reach full speed
     *      - BALL1_FEED_TIME: How long ramp is UP for ball 1
     *      - BALL1_RECOVERY_TIME: Wait time after ball 1 shoots
     *      - BALL2_FEED_TIME: How long ramp is UP for ball 2
     *      - BALL2_RECOVERY_TIME: Wait time after ball 2 shoots
     *      - BALL3_FEED_TIME: How long ramp is UP for ball 3
     *
     *    INTAKE CONTROL (true = ON, false = OFF):
     *      - SPINUP_INTAKE_ON: Intake during spinup phase
     *      - BALL1_FEED_INTAKE_ON: Intake during ball 1 feed
     *      - BALL1_RECOVERY_INTAKE_ON: Intake during ball 1 recovery
     *      - BALL2_FEED_INTAKE_ON: Intake during ball 2 feed
     *      - BALL2_RECOVERY_INTAKE_ON: Intake during ball 2 recovery
     *      - BALL3_FEED_INTAKE_ON: Intake during ball 3 feed
     *      - FINISH_INTAKE_ON: Intake after all balls shot
     *
     *    EJECT CONTROL (true = EJECT, false = NORMAL):
     *      - SPINUP_EJECT_ON: Eject during spinup phase
     *      - BALL1_FEED_EJECT_ON: Eject during ball 1 feed
     *      - BALL1_RECOVERY_EJECT_ON: Eject during ball 1 recovery
     *      - BALL2_FEED_EJECT_ON: Eject during ball 2 feed
     *      - BALL2_RECOVERY_EJECT_ON: Eject during ball 2 recovery
     *      - BALL3_FEED_EJECT_ON: Eject during ball 3 feed
     *      - FINISH_EJECT_ON: Eject after all balls shot
     *
     * 2. The calculated time markers (BALL1_START, BALL1_END, etc.) will
     *    automatically update based on your timing constants
     *
     * SEQUENCE BREAKDOWN:
     * Phase 1: Spinup (0s to SPINUP_TIME)
     *   - Ramp: DOWN
     *   - Intake: Controlled by SPINUP_INTAKE_ON
     *   - Eject: Controlled by SPINUP_EJECT_ON
     *   - Purpose: Let flywheel reach full speed
     *
     * Phase 2: Ball 1 Feed (BALL1_START to BALL1_END)
     *   - Ramp: UP
     *   - Intake: Controlled by BALL1_FEED_INTAKE_ON
     *   - Eject: Controlled by BALL1_FEED_EJECT_ON
     *   - Purpose: Feed ball 1 into flywheel
     *
     * Phase 3: Ball 1 Recovery (BALL1_END to BALL2_START)
     *   - Ramp: DOWN
     *   - Intake: Controlled by BALL1_RECOVERY_INTAKE_ON
     *   - Eject: Controlled by BALL1_RECOVERY_EJECT_ON
     *   - Purpose: Wait for ball 1 to clear, prepare next ball
     *
     * Phase 4: Ball 2 Feed (BALL2_START to BALL2_END)
     *   - Ramp: UP
     *   - Intake: Controlled by BALL2_FEED_INTAKE_ON
     *   - Eject: Controlled by BALL2_FEED_EJECT_ON
     *   - Purpose: Feed ball 2 into flywheel
     *
     * Phase 5: Ball 2 Recovery (BALL2_END to BALL3_START)
     *   - Ramp: DOWN
     *   - Intake: Controlled by BALL2_RECOVERY_INTAKE_ON
     *   - Eject: Controlled by BALL2_RECOVERY_EJECT_ON
     *   - Purpose: Wait for ball 2 to clear, prepare next ball
     *
     * Phase 6: Ball 3 Feed (BALL3_START to BALL3_END)
     *   - Ramp: UP
     *   - Intake: Controlled by BALL3_FEED_INTAKE_ON
     *   - Eject: Controlled by BALL3_FEED_EJECT_ON
     *   - Purpose: Feed ball 3 into flywheel
     *
     * Phase 7: Finish (BALL3_END onwards)
     *   - Ramp: DOWN
     *   - Intake: Controlled by FINISH_INTAKE_ON
     *   - Eject: Controlled by FINISH_EJECT_ON
     *   - Purpose: Shooting complete
     *
     * The flywheel stays at full power throughout the ENTIRE sequence
     */
    private void performShooting() {



        // Keep turret at active position
        if (turretServo != null) {
            turretServo.setPosition(activeTurretPosition);
        }
// Keep flywheel spinning at active power the entire time
// This ensures consistent shot velocity for all 3 balls
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(activeFlywheelPower);  // ✅ CORRECT
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(activeFlywheelPower);  // ✅ CORRECT
        }
        launcher.setHoodPosition(activeHoodPosition);  // ✅ CORRECT
        launcher.setSpinning(true);

        // Get current time in shooting sequence
        double elapsed = shootTimer.getElapsedTimeSeconds();

        // ========== PHASE 1: SPINUP ==========
        // Wait for flywheel to reach full speed before feeding any balls
        if (elapsed < SPINUP_TIME) {
            intakeTransfer.transferDown();  // Ramp DOWN - no balls fed yet

            // Control intake based on SPINUP_INTAKE_ON constant
            if (SPINUP_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (SPINUP_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }

        // ========== PHASE 2: BALL 1 FEED ==========
        // Feed ball 1 into the flywheel
        else if (elapsed >= BALL1_START && elapsed < BALL1_END) {
            intakeTransfer.transferUp();    // Ramp UP - ball can transfer to flywheel

            // Control intake based on BALL1_FEED_INTAKE_ON constant
            if (BALL1_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (BALL1_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }

        // ========== PHASE 3: BALL 1 RECOVERY ==========
        // Wait for ball 1 to fully clear the system
        else if (elapsed >= BALL1_END && elapsed < BALL2_START) {
            intakeTransfer.transferDown();  // Ramp DOWN - prevent premature feeding

            // Control intake based on BALL1_RECOVERY_INTAKE_ON constant
            if (BALL1_RECOVERY_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (BALL1_RECOVERY_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }

        // ========== PHASE 4: BALL 2 FEED ==========
        // Feed ball 2 into the flywheel
        else if (elapsed >= BALL2_START && elapsed < BALL2_END) {
            intakeTransfer.transferUp();    // Ramp UP - ball can transfer to flywheel

            // Control intake based on BALL2_FEED_INTAKE_ON constant
            if (BALL2_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (BALL2_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }

        // ========== PHASE 5: BALL 2 RECOVERY ==========
        // Wait for ball 2 to fully clear the system
        else if (elapsed >= BALL2_END && elapsed < BALL3_START) {
            intakeTransfer.transferDown();  // Ramp DOWN - prevent premature feeding

            // Control intake based on BALL2_RECOVERY_INTAKE_ON constant
            if (BALL2_RECOVERY_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (BALL2_RECOVERY_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }

        // ========== PHASE 6: BALL 3 FEED ==========
        // Feed ball 3 into the flywheel
        else if (elapsed >= BALL3_START && elapsed < BALL3_END) {
            intakeTransfer.transferUp();    // Ramp UP - ball can transfer to flywheel

            // Control intake based on BALL3_FEED_INTAKE_ON constant
            if (BALL3_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (BALL3_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }

        // ========== PHASE 7: FINISH ==========
        // All balls shot - turn off systems
        else if (elapsed >= BALL3_END) {
            intakeTransfer.transferDown();  // Ramp DOWN - shooting complete

            // Control intake based on FINISH_INTAKE_ON constant
            if (FINISH_EJECT_ON) {
                intakeTransfer.startEject(1.0);  // Eject if enabled
            } else if (FINISH_INTAKE_ON) {
                intakeTransfer.startIntake();     // Intake ON if enabled
            } else {
                intakeTransfer.stopIntake();      // Intake OFF if disabled
            }
        }
    }

    /**
     * Stop shooting and turn off all shooter systems
     * Turns off flywheel motors, stops intake, and lowers transfer ramp
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
     * Resets the path timer when switching to a new state
     */
    private void setPathState(PathState newState) {
        if (pathState != newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }
    }

    /**
     * Update telemetry with robot state and debugging information
     * Displays autonomous status, shooter status, and robot position
     */
    private void updateTelemetry() {
        // Display current autonomous state
        telemetry.addLine("=== Autonomous Status ===");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current Speed", String.format("%.0f%%", currentSpeed * 100));
        telemetry.addData("Path Time (s)", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("OpMode Time (s)", String.format("%.2f", opModeTimer.getElapsedTimeSeconds()));

        // Display shooter system status with detailed timing during shooting
        telemetry.addLine();
        telemetry.addLine("=== Shooter Status ===");
        telemetry.addData("Flywheel", launcher.isSpinning() ? "ON" : "OFF");
        telemetry.addData("Hood Position", String.format("%.2f", launcher.getHoodPosition()));
        telemetry.addData("Turret", String.format("%.2f", activeTurretPosition));
        // If currently shooting, display detailed timing information
        if (pathState == PathState.PRELOAD_SHOOTING ||
                pathState == PathState.SHOOTING_SET_1 ||
                pathState == PathState.SHOOTING_SET_2) {
            double elapsed = shootTimer.getElapsedTimeSeconds();
            telemetry.addData("Shoot Timer", String.format("%.2fs", elapsed));

            // Display current phase of shooting sequence
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

        // Display current robot position
        telemetry.addLine();
        telemetry.addLine("=== Robot Position ===");
        Pose currentPose = follower.getPose();
        telemetry.addData("X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(currentPose.getHeading())));

        // Display warning if approaching autonomous timeout
        telemetry.addLine();
        if (opModeTimer.getElapsedTimeSeconds() > 27) {
            telemetry.addLine("WARNING: Approaching 30-second autonomous timeout!");
        }

        telemetry.update();
    }

    /**
     * Stop method - called when autonomous ends
     * Turns off all robot systems and stops path following
     */
    @Override
    public void stop() {
        // Turn off launcher
        if (launcher != null) {
            launcher.setSpinning(false);
        }
        // Turn off intake and lower ramp
        if (intakeTransfer != null) {
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
        }
        // Stop following paths
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
        public PathChain gettingNextSetOfBalls;   // Approach to spike 2
        public PathChain gettingFirstBallSet2;    // Collect ball 1 from spike 2
        public PathChain gettingSecondBallSet2;   // Collect ball 2 from spike 2
        public PathChain gettingThirdBallSet2;    // Collect ball 3 from spike 2
        public PathChain goingBackToShootSet2;    // Return to shooting position

        /**
         * Constructor - Build all paths using pose constants
         * Paths are built using either BezierLine (straight) or BezierCurve (curved)
         * Heading interpolation determines how the robot rotates during the path:
         * - setConstantHeadingInterpolation: Robot maintains constant heading
         * - setLinearHeadingInterpolation: Robot heading changes linearly
         * - setTangentHeadingInterpolation: Robot heading follows path tangent
         */
        public Paths(Follower follower) {
            // ========== FIRST BALL SET PATHS ==========

            // Path 1: Curved approach from starting position to spike 1 approach point
            goingToNearestBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(24.143, 125.525),
                            SPIKE1_CURVE_CONTROL,
                            SPIKE1_APPROACH
                    ))
                    .setLinearHeadingInterpolation(HEADING_233, HEADING_180)
                    .build();

            // Path 2: Slow precise movement to first ball
            gettingFirstBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_APPROACH, SPIKE1_BALL1))
                    .setTangentHeadingInterpolation()
                    .build();

            // Path 3: Slow precise movement to second ball
            gettingSecondBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL1, SPIKE1_BALL2))
                    .setTangentHeadingInterpolation()
                    .build();

            // Path 4: Slow precise movement to third ball
            gettingThirdBallSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL2, SPIKE1_BALL3))
                    .setTangentHeadingInterpolation()
                    .build();

            // Path 5: Fast return to shooting position
            goingBackToShootSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE1_BALL3, SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            // ========== SECOND BALL SET PATHS ==========

            // Path 6: Straight line approach to spike 2
            gettingNextSetOfBalls = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, SPIKE2_APPROACH))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();

            // Path 7: Slow precise movement to first ball of spike 2
            gettingFirstBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_APPROACH, SPIKE2_BALL1))
                    .setTangentHeadingInterpolation()
                    .build();

            // Path 8: Slow precise movement to second ball of spike 2
            gettingSecondBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL1, SPIKE2_BALL2))
                    .setTangentHeadingInterpolation()
                    .build();

            // Path 9: Slow precise movement to third ball of spike 2
            gettingThirdBallSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL2, SPIKE2_BALL3))
                    .setTangentHeadingInterpolation()
                    .build();

            // Path 10: Fast return to final shooting position
            goingBackToShootSet2 = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE2_BALL3, FINAL_SHOOT_POSE))
                    .setConstantHeadingInterpolation(HEADING_180)
                    .build();
        }
    }
}
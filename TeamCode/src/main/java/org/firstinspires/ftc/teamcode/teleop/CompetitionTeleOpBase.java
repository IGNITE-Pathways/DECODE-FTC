package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.ActualTurretLockOn;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.core.util.DistanceCalculator;

/**
 * *** COMPETITION TELEOP ***
 *
 * ALL SETTINGS (PIDF gains, RPM ranges, hood positions) are in RobotConstants.java!
 * After tuning, update values there - they automatically apply here.
 *
 * === DEFAULT PRESET ===
 * Default: FAR ZONE (3450 RPM, hood 0.75) - optimized for back wall cycling (10+ ft)
 *
 * === FLYWHEEL & RAMP CONTROL (NEW!) ===
 * Flywheel and ramp are NOW SEPARATE:
 * - GP1-Y: Toggle flywheel ON/OFF (does NOT raise ramp)
 * - GP2-B: Raise/lower ramp (ONLY works when flywheel at speed, within 100 RPM)
 * - You can intake/cycle balls while flywheel is spinning!
 *
 * === GAMEPAD 1 (Driver/Shooter/Presets) ===
 * Sticks: Drive (100% speed always)
 * Y: Toggle flywheel ON/OFF (does NOT raise ramp)
 * RB: Start auto-shoot 3-ball sequence (when flywheel ON) | Press during auto = Cancel
 * B: Lock distance (when distance detection enabled)
 * LB: Eject-and-shoot sequence (gentle eject 400ms, then shoot 800ms)
 * RT: Intake (works even when flywheel ON!) | LT: Eject
 * DPAD UP: Enable distance detection
 * DPAD DOWN: Disable distance detection (use FAR ZONE default)
 * DPAD RIGHT: Far zone preset (10+ ft, 3450 RPM)
 *
 * === GAMEPAD 2 (Turret & Manual Controls) ===
 * A: Turret auto-track (aims at AprilTag)
 * B: Raise/lower ramp (ONLY if flywheel at speed!)
 * X: Center turret
 * Left Stick X: Manual turret control
 * DPAD LEFT: Fine turret adjust left
 * DPAD UP: Increase RPM (+100)
 * DPAD DOWN: Decrease RPM (-100)
 * LB: Lower hood angle (-0.03)
 * RB: Raise hood angle (+0.03)
 * START: EMERGENCY STOP
 *
 * === Shooting Workflow (Back Wall Cycling) ===
 * 1. Position at back wall (default is FAR ZONE - 3450 RPM)
 * 2. GP2-A: Enable turret tracking (aims at AprilTag)
 * 3. GP1-Y: Turn ON flywheel (spins up, ramp stays DOWN)
 * 4. GP1-RT: Intake balls while flywheel spins up
 * 5. GP2-B: Raise ramp when ready to shoot (only works if RPM ready)
 * 6. GP1-RT: Feed balls to shoot
 * 7. GP2-B: Lower ramp to cycle more balls
 * 8. Repeat steps 4-7 for continuous cycling!
 *
 * === Quick Eject-Shoot (GP1-LB) ===
 * Use when ball is stuck or needs repositioning before shooting
 * - Automatically switches to SHOOTING mode if needed
 * - Auto-raises ramp if flywheel at speed
 * - Gentle eject (35% power, 400ms) to reposition ball properly
 * - Then shoots for 800ms
 */
public abstract class CompetitionTeleOpBase extends LinearOpMode {

    // === STATE MACHINE ===
    private enum RobotState {
        INTAKE,   // Full speed, intake active
        SHOOTING  // Reduced speed, flywheel active
    }

    // === CONFIG ===
    private static final boolean SHOW_DEBUG_INFO = false;  // Set true to see limelight debug info

    protected AllianceColor alliance;
    private RobotState currentState = RobotState.INTAKE;  // Start in intake mode

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private ActualTurretLockOn turret;
    private Limelight3A limelight;

    // State
    private boolean flywheelOn = false;
    private boolean rampUp = false;  // NEW: Separate ramp control from flywheel
    private double flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;  // Loaded from ShooterConstants
    private double hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
    private String selectedPreset = "DEFAULT";
    private boolean turretTracking = false;
    private boolean distanceDetectionEnabled = false;  // Default: OFF
    private boolean emergencyStopped = false;

    // Gamepad 1 button states
    private boolean lastGP1_Y = false;
    private boolean lastGP1_B = false;
    private boolean lastGP1_LeftBumper = false;
    private boolean lastGP1_RightBumper = false;
    private boolean lastGP1_DpadUp = false;
    private boolean lastGP1_DpadDown = false;
    private boolean lastGP1_DpadRight = false;

    // Eject-and-shoot sequence
    private boolean ejectShootActive = false;
    private long ejectShootStartTime = 0;
    private static final long EJECT_DURATION_MS = 400;  // Eject for 400ms to position ball properly

    // Auto-shoot sequence (triggered by Y button in SHOOTING mode) - 3 BALLS
    private boolean autoShootActive = false;
    private boolean waitingForFlywheelSpinup = false;
    private int autoShootPhase = 0;  // 0=initial wait, 1=shot1, 2=wait, 3=shot2, 4=wait, 5=shot3
    private ElapsedTime autoShootTimer = new ElapsedTime();
    private static final double RPM_TOLERANCE = 100;  // Within 100 RPM to start/continue shooting
    private static final double SHOT_DURATION_MS = 500;   // 0.5 seconds for shots 1 & 2
    private static final double FINAL_SHOT_DURATION_MS = 1000;  // 1.0 second for final shot (ball 3)

    // Limelight distance tracking
    private double currentDistance = -1.0;  // Currently detected distance
    private double lockedDistance = -1.0;   // Locked distance for shooting

    // Gamepad 2 button states (manual turret & overrides)
    private boolean lastGP2_A = false;
    private boolean lastGP2_B = false;  // NOW USED FOR RAMP CONTROL
    private boolean lastGP2_X = false;
    private boolean lastGP2_DpadLeft = false;
    private boolean lastGP2_DpadRight = false;
    private boolean lastGP2_DpadUp = false;
    private boolean lastGP2_DpadDown = false;
    private boolean lastGP2_LeftBumper = false;
    private boolean lastGP2_RightBumper = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            // Emergency stop check (GP2-START)
            handleEmergencyStop();

            if (!emergencyStopped) {
                // Main control loop
                handleDriving();              // GP1: Sticks, speed control
                handleTurret();               // GP2-A: Auto-tracking toggle
                handleManualTurretControl();  // GP2: Manual turret control
                handleIntake();               // GP1: RT/LT intake/eject
                handleEjectShoot();           // GP1-LB: Eject-and-shoot sequence
                handleDistanceLock();         // GP2: DPAD UP/DOWN, Y-far, B-lock
                handleFlywheelToggle();       // GP1-Y: Toggle flywheel ON/OFF
                handleRampControl();          // GP2-B: Manual ramp control with RPM check
                handleAutoShootTrigger();     // GP1-RB: Trigger auto-shoot sequence
                handleAutoShootSequence();    // Auto-shoot sequence (3-ball)
                handleManualOverrides();      // GP2: Manual hood adjustments
                launcher.update();            // PIDF velocity control
            }

            updateTelemetry();
        }

        shutdown();
    }

    // ========================================
    // INITIALIZATION
    // ========================================

    private void initializeHardware() {
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        // Apply driver comfort settings from ShooterConstants
        driveTrain.setInputCurve(RobotConstants.DRIVE_INPUT_CURVE);
        driveTrain.setRotationSensitivity(RobotConstants.ROTATION_SENSITIVITY);
        driveTrain.setBatteryCompensationEnabled(RobotConstants.ENABLE_BATTERY_COMPENSATION);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // PIDF velocity control enabled/disabled from ShooterConstants
        // Tune gains using FlywheelPIDFTuner, then update RobotConstants.java
        launcher.setVelocityControlEnabled(RobotConstants.USE_VELOCITY_CONTROL);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        turret = new ActualTurretLockOn();
        turret.initialize(hardwareMap, telemetry, alliance);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(20);
            limelight.pipelineSwitch(3);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        telemetry.addLine(alliance.name() + " TeleOp Ready");
        telemetry.addLine(limelight != null ? "Limelight: OK" : "Limelight: NOT FOUND");
        telemetry.update();
    }

    // GP2-START: Emergency stop (hold to stop all motors, release to resume)
    private void handleEmergencyStop() {
        if (gamepad2.start) {
            if (!emergencyStopped) {
                emergencyStopped = true;
                stopAllMotors();
            }
        } else {
            emergencyStopped = false;
        }
    }

    private void stopAllMotors() {
        launcher.setSpinning(false);
        launcher.setTargetRPM(0);
        intakeTransfer.stopIntake();
        intakeTransfer.transferDown();
        driveTrain.stopMotors();
        turret.stop();
        turretTracking = false;
        flywheelOn = false;
        rampUp = false;  // Reset ramp state
    }

    // ========================================
    // GAMEPAD 1: DRIVING & INTAKE
    // ========================================

    private void handleDriving() {
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Apply joystick deadzone
        fwd = applyDeadzone(fwd);
        str = applyDeadzone(str);
        rot = applyDeadzone(rot);

        // Apply input curve to translation vector magnitude (preserves direction)
        double magnitude = Math.sqrt(fwd * fwd + str * str);
        if (magnitude > 0.01) {
            double curvedMagnitude = Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
            double scale = curvedMagnitude / magnitude;
            fwd *= scale;
            str *= scale;
        }

        // Apply input curve to rotation separately
        rot = applyInputCurve(rot);

        // Apply rotation sensitivity
        rot *= RobotConstants.ROTATION_SENSITIVITY;

        // Full speed for both modes - move fast during game
        double speedMult = 1.0;  // 100% speed always

        driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);
    }

    private double applyInputCurve(double input) {
        if (input == 0) return 0;
        double sign = Math.signum(input);
        double magnitude = Math.abs(input);
        return sign * Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < RobotConstants.JOYSTICK_DEAD_ZONE) {
            return 0;
        }
        return input;
    }

    // GP2-A: Toggle turret auto-tracking (limelight aims at AprilTag)
    private void handleTurret() {
        if (gamepad2.a && !lastGP2_A) {
            turretTracking = !turretTracking;
        }
        lastGP2_A = gamepad2.a;

        if (turretTracking) {
            turret.update();  // Auto-track AprilTag
        }
    }

    // GP2: Manual turret control (only when auto-tracking is OFF)
    private void handleManualTurretControl() {
        if (turretTracking) {
            return;  // Auto-tracking takes priority
        }

        double currentPos = turret.getServoPosition();

        // Gamepad 2 Left Stick X: Continuous manual control
        double stickInput = -gamepad2.left_stick_x;  // Inverted for intuitive control
        if (Math.abs(stickInput) > RobotConstants.TURRET_MANUAL_STICK_DEADZONE) {
            double adjustment = stickInput * RobotConstants.TURRET_MANUAL_STICK_SENSITIVITY;
            currentPos += adjustment;
        }

        // Gamepad 2 DPAD Left: Small increment left
        if (gamepad2.dpad_left && !lastGP2_DpadLeft) {
            currentPos -= RobotConstants.TURRET_MANUAL_INCREMENT;
        }
        lastGP2_DpadLeft = gamepad2.dpad_left;

        // Gamepad 2 X: Reset to center
        if (gamepad2.x && !lastGP2_X) {
            currentPos = RobotConstants.TURRET_CENTER_POSITION;
        }
        lastGP2_X = gamepad2.x;

        // Clamp position to valid range
        currentPos = Math.max(RobotConstants.TURRET_MIN_POSITION,
                             Math.min(RobotConstants.TURRET_MAX_POSITION, currentPos));

        // Apply position
        turret.setPositionDirect(currentPos);
    }

    // GP1: RT = Intake, LT = Eject
    private void handleIntake() {
        // Don't allow manual intake during auto sequences
        if (ejectShootActive || autoShootActive) {
            return;
        }

        if (gamepad1.right_trigger > RobotConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > RobotConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
        }
    }

    // GP1-LB: Eject ball halfway then shoot
    private void handleEjectShoot() {
        // Start sequence on button press
        if (gamepad1.left_bumper && !lastGP1_LeftBumper) {
            ejectShootActive = true;
            ejectShootStartTime = System.currentTimeMillis();

            // Ensure flywheel is ready
            if (!flywheelOn) {
                currentState = RobotState.SHOOTING;
                activateFlywheel();
            }

            // Automatically raise ramp if flywheel is at speed
            if (flywheelOn && !rampUp) {
                double currentRPM = launcher.getCurrentRPM();
                double error = Math.abs(flywheelRPM - currentRPM);
                if (error <= 100) {
                    intakeTransfer.transferUp();
                    rampUp = true;
                }
            }
        }
        lastGP1_LeftBumper = gamepad1.left_bumper;

        // Execute sequence
        if (ejectShootActive) {
            long elapsed = System.currentTimeMillis() - ejectShootStartTime;

            if (elapsed < EJECT_DURATION_MS) {
                // Phase 1: Eject to position ball (reverse intake at reduced power)
                intakeTransfer.startEject(0.35);  // Low power eject to just reposition (was 1.0)
            } else if (elapsed < EJECT_DURATION_MS + 800) {
                // Phase 2: Shoot (forward intake to feed into flywheel)
                intakeTransfer.startIntake(1.0);  // Full power shoot
            } else {
                // Sequence complete
                intakeTransfer.stopIntake();
                ejectShootActive = false;
            }
        }
    }

    // ========================================
    // GAMEPAD 2: LAUNCHER CONTROLS
    // ========================================

    // GP1 DPAD: Distance detection & presets | GP1-B: Lock distance
    private void handleDistanceLock() {
        // DPAD UP: Enable limelight distance detection
        if (gamepad1.dpad_up && !lastGP1_DpadUp) {
            distanceDetectionEnabled = true;
        }
        lastGP1_DpadUp = gamepad1.dpad_up;

        // DPAD DOWN: Disable detection, use default preset
        if (gamepad1.dpad_down && !lastGP1_DpadDown) {
            distanceDetectionEnabled = false;
            currentDistance = -1.0;
            lockedDistance = -1.0;
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
            selectedPreset = "DEFAULT";
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
                launcher.setHoodPosition(hoodPosition);
            }
        }
        lastGP1_DpadDown = gamepad1.dpad_down;

        // DPAD RIGHT: Quick far zone preset (10+ ft, max distance)
        if (gamepad1.dpad_right && !lastGP1_DpadRight) {
            distanceDetectionEnabled = false;
            lockedDistance = RobotConstants.RANGE_FAR_MIN;
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
            selectedPreset = "FAR ZONE (Manual)";
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
                launcher.setHoodPosition(hoodPosition);
            }
        }
        lastGP1_DpadRight = gamepad1.dpad_right;

        // Continuously read distance from limelight when enabled
        if (distanceDetectionEnabled) {
            double distance = calculateCurrentDistance();
            if (distance > 0) {
                currentDistance = distance;
            }
        }

        // B: Lock current distance and calculate shooting preset
        if (gamepad1.b && !lastGP1_B) {
            if (distanceDetectionEnabled && currentDistance > 0) {
                lockedDistance = currentDistance;
                applyDistancePreset(lockedDistance);  // Sets RPM & hood for this distance
            }
        }
        lastGP1_B = gamepad1.b;
    }

    private double calculateCurrentDistance() {
        if (limelight == null) {
            return -1.0;
        }

        if (!limelight.isConnected()) {
            return -1.0;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return -1.0;
        }

        return DistanceCalculator.calculateDistance(result);
    }

    // Debug info for limelight
    private String getLimelightDebugInfo() {
        if (limelight == null) {
            return "Limelight: NULL";
        }
        if (!limelight.isConnected()) {
            return "Limelight: NOT CONNECTED";
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return "Limelight: NO RESULT";
        }
        if (!result.isValid()) {
            return "Limelight: INVALID RESULT";
        }

        double ta = result.getTa();
        return String.format("Limelight OK | TA: %.2f%%", ta);
    }

    private void applyDistancePreset(double distance) {
        // All range values loaded from RobotConstants - update there after testing!
        // Full coverage: 2.45-3.45ft, 3.46-4.65ft, 4.66-5.35ft, 5.36-6ft, 6-7ft, 7-8.5ft, 8.5-10ft, 10+ft

        if (distance >= RobotConstants.RANGE_1_MIN && distance <= RobotConstants.RANGE_1_MAX) {
            // Range 1: 2.45 - 3.45 ft
            flywheelRPM = RobotConstants.RANGE_1_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_1_HOOD_POSITION;
            selectedPreset = String.format("RANGE 1 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_2_MIN && distance <= RobotConstants.RANGE_2_MAX) {
            // Range 2: 3.46 - 4.65 ft
            flywheelRPM = RobotConstants.RANGE_2_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_2_HOOD_POSITION;
            selectedPreset = String.format("RANGE 2 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_3_MIN && distance <= RobotConstants.RANGE_3_MAX) {
            // Range 3: 4.66 - 5.35 ft
            flywheelRPM = RobotConstants.RANGE_3_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_3_HOOD_POSITION;
            selectedPreset = String.format("RANGE 3 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_4_MIN && distance <= RobotConstants.RANGE_4_MAX) {
            // Range 4: 5.36 - 6.00 ft
            flywheelRPM = RobotConstants.RANGE_4_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_4_HOOD_POSITION;
            selectedPreset = String.format("RANGE 4 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_5_MIN && distance <= RobotConstants.RANGE_5_MAX) {
            // Range 5: 6.01 - 7.00 ft
            flywheelRPM = RobotConstants.RANGE_5_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_5_HOOD_POSITION;
            selectedPreset = String.format("RANGE 5 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_6_MIN && distance <= RobotConstants.RANGE_6_MAX) {
            // Range 6: 7.01 - 8.50 ft
            flywheelRPM = RobotConstants.RANGE_6_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_6_HOOD_POSITION;
            selectedPreset = String.format("RANGE 6 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_7_MIN && distance <= RobotConstants.RANGE_7_MAX) {
            // Range 7: 8.51 - 10.00 ft
            flywheelRPM = RobotConstants.RANGE_7_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_7_HOOD_POSITION;
            selectedPreset = String.format("RANGE 7 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_FAR_MIN) {
            // Far range: 10+ ft
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
            selectedPreset = String.format("FAR ZONE (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else {
            // Below 2.45 ft or invalid distance - use defaults
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
            selectedPreset = String.format("OUT OF RANGE (%.2fft) - DEFAULT", distance);
        }

        // Apply immediately if flywheel is already on
        if (flywheelOn) {
            launcher.setTargetRPM(flywheelRPM);
            launcher.setHoodPosition(hoodPosition);
        }
    }

    // GP1-Y: Toggle flywheel ON/OFF
    // Turns on flywheel and enters SHOOTING mode, or turns off and returns to INTAKE mode
    private void handleFlywheelToggle() {
        if (gamepad1.y && !lastGP1_Y) {
            if (currentState == RobotState.INTAKE) {
                // Switch to SHOOTING mode
                currentState = RobotState.SHOOTING;
                activateFlywheel();  // Spin up to target RPM, raise transfer ramp
            } else if (currentState == RobotState.SHOOTING) {
                // Turn off flywheel and return to INTAKE mode
                if (autoShootActive) {
                    cancelAutoShoot();
                }
                currentState = RobotState.INTAKE;
                deactivateFlywheel();
            }
        }
        lastGP1_Y = gamepad1.y;
    }

    // GP1-RB: Trigger auto-shoot sequence
    // Starts the 3-ball auto-shoot sequence (only works when flywheel is ON)
    private void handleAutoShootTrigger() {
        if (gamepad1.right_bumper && !lastGP1_RightBumper) {
            if (currentState == RobotState.SHOOTING && !autoShootActive) {
                // Start auto-shoot sequence
                startAutoShoot();
            } else if (autoShootActive) {
                // Cancel auto-shoot if already running
                cancelAutoShoot();
            }
        }
        lastGP1_RightBumper = gamepad1.right_bumper;
    }

    private void activateFlywheel() {
        launcher.setTargetRPM(flywheelRPM);  // Set target RPM for PIDF control
        launcher.setSpinning(true);
        launcher.setHoodPosition(hoodPosition);
        // REMOVED: intakeTransfer.transferUp() - ramp now controlled separately via GP2-B
        flywheelOn = true;
    }

    private void deactivateFlywheel() {
        launcher.setSpinning(false);
        intakeTransfer.transferDown();  // Always lower ramp when flywheel turns off
        rampUp = false;  // Reset ramp state
        flywheelOn = false;
    }

    // GP2-B: Manual ramp control with RPM check
    // Only allows ramp to go up if flywheel is at speed (within 50-100 RPM of target)
    private void handleRampControl() {
        if (gamepad2.b && !lastGP2_B) {
            if (!rampUp) {
                // Try to raise ramp
                if (flywheelOn) {
                    double currentRPM = launcher.getCurrentRPM();
                    double error = Math.abs(flywheelRPM - currentRPM);

                    // Check if flywheel is at speed (within 100 RPM tolerance)
                    if (error <= 100) {
                        // RPM good! Raise ramp
                        intakeTransfer.transferUp();
                        rampUp = true;
                    } else {
                        // RPM not ready yet - don't raise ramp
                        // Telemetry will show this in updateTelemetry
                    }
                } else {
                    // Flywheel not even on - can't raise ramp
                }
            } else {
                // Lower ramp
                intakeTransfer.transferDown();
                rampUp = false;
            }
        }
        lastGP2_B = gamepad2.b;
    }

    // ========================================
    // AUTO-SHOOT SEQUENCE (3-BALL)
    // ========================================

    /**
     * Start the auto-shoot sequence
     * Waits for flywheel to reach target RPM, then fires 3 balls automatically
     * Waits for RPM recovery between each shot for consistency
     */
    private void startAutoShoot() {
        autoShootActive = true;
        waitingForFlywheelSpinup = true;
        autoShootPhase = 0;
        autoShootTimer.reset();
        intakeTransfer.stopIntake();  // Stop any ongoing intake

        // Automatically raise ramp if flywheel at speed
        if (!rampUp) {
            double currentRPM = launcher.getCurrentRPM();
            double error = Math.abs(flywheelRPM - currentRPM);
            if (error <= 100) {
                intakeTransfer.transferUp();
                rampUp = true;
            }
        }
    }

    /**
     * Cancel the auto-shoot sequence
     */
    private void cancelAutoShoot() {
        autoShootActive = false;
        waitingForFlywheelSpinup = false;
        autoShootPhase = 0;
        intakeTransfer.stopIntake();
    }

    /**
     * Handle the auto-shoot sequence timing
     * Sequence: Wait for RPM → Ball 1 (0.5s) → Wait for RPM → Ball 2 (0.5s) → Wait for RPM → Ball 3 (1.0s) → Done
     */
    private void handleAutoShootSequence() {
        if (!autoShootActive) {
            return;
        }

        double currentRPM = launcher.getCurrentRPM();
        double targetRPM = launcher.getTargetRPM();
        double error = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = error <= RPM_TOLERANCE;

        // Phase 0: Wait for initial flywheel spinup
        if (waitingForFlywheelSpinup) {
            if (rpmReady) {
                // Flywheel is ready! Start shooting sequence
                waitingForFlywheelSpinup = false;
                autoShootPhase = 1;
                autoShootTimer.reset();
            }
            return;  // Keep waiting
        }

        // Execute shooting sequence with RPM-based recovery
        double elapsed = autoShootTimer.milliseconds();

        switch (autoShootPhase) {
            case 1:  // Ball 1: Shoot for 0.5s
                if (elapsed < SHOT_DURATION_MS) {
                    intakeTransfer.startIntake(1.0);
                } else {
                    intakeTransfer.stopIntake();
                    autoShootPhase = 2;  // Move to recovery wait
                    autoShootTimer.reset();
                }
                break;

            case 2:  // Wait for RPM recovery after ball 1
                if (rpmReady) {
                    autoShootPhase = 3;  // RPM recovered, shoot ball 2
                    autoShootTimer.reset();
                }
                // Keep waiting for RPM to recover
                break;

            case 3:  // Ball 2: Shoot for 0.5s
                if (elapsed < SHOT_DURATION_MS) {
                    intakeTransfer.startIntake(1.0);
                } else {
                    intakeTransfer.stopIntake();
                    autoShootPhase = 4;  // Move to recovery wait
                    autoShootTimer.reset();
                }
                break;

            case 4:  // Wait for RPM recovery after ball 2
                if (rpmReady) {
                    autoShootPhase = 5;  // RPM recovered, shoot ball 3 (final)
                    autoShootTimer.reset();
                }
                // Keep waiting for RPM to recover
                break;

            case 5:  // Ball 3 (FINAL): Shoot for 1.0s
                if (elapsed < FINAL_SHOT_DURATION_MS) {
                    intakeTransfer.startIntake(1.0);
                } else {
                    // Sequence complete!
                    intakeTransfer.stopIntake();
                    autoShootActive = false;
                    autoShootPhase = 0;
                }
                break;
        }
    }

    // GP2 Manual Overrides:
    // - DPAD UP/DOWN: Adjust RPM (±100 RPM per press)
    // - LEFT/RIGHT BUMPER: Adjust hood angle (±0.03 per press)
    // - START: Emergency stop
    private void handleManualOverrides() {
        // DPAD UP: Increase RPM by 100
        if (gamepad2.dpad_up && !lastGP2_DpadUp) {
            flywheelRPM += 100;
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
            }
            selectedPreset = String.format("MANUAL (%.0f RPM)", flywheelRPM);
        }
        lastGP2_DpadUp = gamepad2.dpad_up;

        // DPAD DOWN: Decrease RPM by 100
        if (gamepad2.dpad_down && !lastGP2_DpadDown) {
            flywheelRPM = Math.max(1000, flywheelRPM - 100);
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
            }
            selectedPreset = String.format("MANUAL (%.0f RPM)", flywheelRPM);
        }
        lastGP2_DpadDown = gamepad2.dpad_down;

        // LEFT BUMPER: Lower hood (decrease angle, flatter trajectory)
        if (gamepad2.left_bumper && !lastGP2_LeftBumper) {
            hoodPosition = Math.max(0.0, hoodPosition - 0.03);
            launcher.setHoodPosition(hoodPosition);
        }
        lastGP2_LeftBumper = gamepad2.left_bumper;

        // RIGHT BUMPER: Raise hood (increase angle, steeper trajectory)
        if (gamepad2.right_bumper && !lastGP2_RightBumper) {
            hoodPosition = Math.min(1.0, hoodPosition + 0.03);
            launcher.setHoodPosition(hoodPosition);
        }
        lastGP2_RightBumper = gamepad2.right_bumper;
    }

    // ========================================
    // TELEMETRY & SHUTDOWN
    // ========================================

    private void updateTelemetry() {
        // === HEADER ===
        telemetry.addLine("=== " + alliance.name() + " TELEOP ===");

        // === ROBOT STATE ===
        telemetry.addData("State", "%s (Speed: 100%%)", currentState);

        // === EJECT-SHOOT SEQUENCE ===
        if (ejectShootActive) {
            long elapsed = System.currentTimeMillis() - ejectShootStartTime;
            if (elapsed < EJECT_DURATION_MS) {
                telemetry.addLine(">>> EJECTING BALL <<<");
            } else {
                telemetry.addLine(">>> SHOOTING BALL <<<");
            }
        }

        // === AUTO-SHOOT SEQUENCE (3-BALL) ===
        if (autoShootActive) {
            if (waitingForFlywheelSpinup) {
                double currentRPM = launcher.getCurrentRPM();
                double targetRPM = launcher.getTargetRPM();
                double error = Math.abs(targetRPM - currentRPM);
                telemetry.addLine(String.format(">>> WAITING FOR FLYWHEEL: %.0f RPM (%.0f to go) <<<",
                    currentRPM, error));
            } else {
                // Phase 1=shoot1, 2=wait, 3=shoot2, 4=wait, 5=shoot3
                String status;
                switch (autoShootPhase) {
                    case 1: status = "BALL 1/3 - SHOOTING"; break;
                    case 2: status = "RECOVERING RPM..."; break;
                    case 3: status = "BALL 2/3 - SHOOTING"; break;
                    case 4: status = "RECOVERING RPM..."; break;
                    case 5: status = "BALL 3/3 - FINAL SHOT"; break;
                    default: status = "AUTO-SHOOT"; break;
                }
                telemetry.addLine(String.format(">>> AUTO-SHOOT: %s <<<", status));
            }
        }

        // === EMERGENCY STOP WARNING ===
        if (emergencyStopped) {
            telemetry.addLine("*** E-STOP ACTIVE - Release GP2-START ***");
        }

        // === SHOOTING STATUS ===
        if (flywheelOn) {
            double currentRPM = launcher.getCurrentRPM();
            double error = Math.abs(flywheelRPM - currentRPM);
            boolean ready = error < 50;  // Ready if within 50 RPM
            boolean boosting = launcher.isInBoostPhase();

            String status;
            if (boosting) {
                status = "BOOST SPINUP...";
            } else if (ready) {
                status = ">>> READY TO SHOOT <<<";
            } else {
                status = "SPINNING UP...";
            }
            telemetry.addData("Flywheel", status);
            telemetry.addData("Target/Actual", "%.0f / %.0f RPM", flywheelRPM, currentRPM);

            // Gamepad rumble when ready
            if (ready && error < 30) {
                gamepad1.rumble(100);  // Quick pulse when locked on speed
            }
        } else {
            telemetry.addData("Flywheel", "OFF (GP1-Y to turn on)");
        }

        // === RAMP STATUS (NEW!) ===
        if (rampUp) {
            telemetry.addData("Ramp", "UP (ready to shoot)");
        } else {
            if (flywheelOn) {
                double currentRPM = launcher.getCurrentRPM();
                double error = Math.abs(flywheelRPM - currentRPM);
                if (error <= 100) {
                    telemetry.addData("Ramp", "DOWN (GP2-B to raise - RPM READY!)");
                } else {
                    telemetry.addData("Ramp", "DOWN (waiting for RPM... %.0f to go)", error);
                }
            } else {
                telemetry.addData("Ramp", "DOWN");
            }
        }

        telemetry.addData("Hood", "%.2f", hoodPosition);
        telemetry.addData("Preset", selectedPreset);

        // === DISTANCE INFO ===
        telemetry.addData("Distance Detect", distanceDetectionEnabled ? "ON" : "OFF");
        if (distanceDetectionEnabled && currentDistance > 0) {
            telemetry.addData("Current Dist", "%.2fft", currentDistance);
        }
        if (lockedDistance > 0) {
            telemetry.addData("Locked Dist", "%.2fft", lockedDistance);
        }

        // === TURRET STATUS ===
        String turretMode = turretTracking ? (turret.isLocked() ? "LOCKED" : "TRACKING") : "MANUAL";
        telemetry.addData("Turret", "%s (%.2f)", turretMode, turret.getServoPosition());

        // === DEBUG INFO (Optional) ===
        if (SHOW_DEBUG_INFO) {
            telemetry.addLine();
            telemetry.addData("Limelight", getLimelightDebugInfo());
        }

        // === CONTROLS (Compact) ===
        telemetry.addLine();
        telemetry.addLine("GP1: Y-Flywheel RB-AutoShoot B-Lock LB-EjectShoot | RT-In LT-Out | DPAD:U/D-Dist R-Far | Sticks-Drive");
        telemetry.addLine("GP2: A-Track B-RAMP X-CtrTur | LS-Turret DPAD:L-Tur U/D-RPM | LB/RB-Hood | START-ESTOP");

        telemetry.update();
    }

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
        turret.stop();
    }
}

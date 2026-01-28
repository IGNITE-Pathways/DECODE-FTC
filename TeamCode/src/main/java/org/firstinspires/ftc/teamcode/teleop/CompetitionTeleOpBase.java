package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Default: Range 1 close shot (2400 RPM, hood 0.55) - optimized for 2.45-3.45 ft
 *
 * === STATE MACHINE ===
 * INTAKE Mode: 100% speed, flywheel OFF
 * SHOOTING Mode: 100% speed, flywheel ON
 * Toggle: GP1-Y switches between modes
 *
 * === GAMEPAD 1 (Driver/Shooter/Presets) ===
 * Sticks: Drive (100% speed always)
 * Y: Toggle INTAKE/SHOOTING mode
 * B: Lock distance (when distance detection enabled)
 * LB: Eject-and-shoot sequence (gentle eject 150ms, then shoot 800ms)
 * RT: Intake | LT: Eject
 * DPAD UP: Enable distance detection
 * DPAD DOWN: Disable distance detection (use Range 1 default)
 * DPAD RIGHT: Far zone preset (10+ ft, 3400 RPM)
 *
 * === GAMEPAD 2 (Turret & Manual Controls) ===
 * A: Turret auto-track (aims at AprilTag)
 * X: Center turret
 * Left Stick X: Manual turret control
 * DPAD LEFT: Fine turret adjust left
 * DPAD UP: Increase RPM (+100)
 * DPAD DOWN: Decrease RPM (-100)
 * LB: Lower hood angle (-0.03)
 * RB: Raise hood angle (+0.03)
 * START: EMERGENCY STOP
 *
 * === Shooting Workflow (Normal) ===
 * 1. GP2-A: Enable turret tracking (aims at AprilTag)
 * 2. GP1-DPAD UP: Enable distance detection
 * 3. GP1-B: Lock distance when lined up
 * 4. GP1-Y: Switch to SHOOTING mode (spins up flywheel)
 * 5. GP1-RT: Feed balls
 * 6. GP2-DPAD UP/DOWN or LB/RB: Fine-tune RPM/hood if needed
 * 7. GP1-Y: Switch back to INTAKE mode when done
 *
 * === Quick Eject-Shoot (GP1-LB) ===
 * Use when ball is stuck or needs repositioning before shooting
 * - Automatically switches to SHOOTING mode if needed
 * - Gentle eject (35% power, 150ms) to reposition ball
 * - Then shoots for 800ms
 * - Hands-free sequence, just press LB once
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
    private boolean lastGP1_DpadUp = false;
    private boolean lastGP1_DpadDown = false;
    private boolean lastGP1_DpadRight = false;

    // Eject-and-shoot sequence
    private boolean ejectShootActive = false;
    private long ejectShootStartTime = 0;
    private static final long EJECT_DURATION_MS = 150;  // Eject for 150ms to position ball (reduced from 300ms)

    // Limelight distance tracking
    private double currentDistance = -1.0;  // Currently detected distance
    private double lockedDistance = -1.0;   // Locked distance for shooting

    // Gamepad 2 button states (manual turret & overrides)
    private boolean lastGP2_A = false;
    private boolean lastGP2_B = false;
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
                handleFlywheelToggle();       // GP1-Y: State toggle
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
        // Don't allow manual intake during eject-shoot sequence
        if (ejectShootActive) {
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

            // Ensure flywheel is ready and transfer is up
            if (!flywheelOn) {
                currentState = RobotState.SHOOTING;
                activateFlywheel();
            } else {
                // Make sure transfer is up if already in shooting mode
                intakeTransfer.transferUp();
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
        // Ranges: 2.45-3.45ft, 3.46-4.65ft, 4.66-5.35ft, 5.36-6ft, 10+ft

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

        } else if (distance >= RobotConstants.RANGE_FAR_MIN) {
            // Far range: 10+ ft
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
            selectedPreset = String.format("FAR ZONE (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else {
            // Between 6-10 ft or below 2.45 ft - use defaults
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

    // GP1-Y: Toggle between INTAKE and SHOOTING states
    private void handleFlywheelToggle() {
        if (gamepad1.y && !lastGP1_Y) {
            if (currentState == RobotState.INTAKE) {
                // Switch to SHOOTING mode
                currentState = RobotState.SHOOTING;
                activateFlywheel();  // Spin up to target RPM, raise transfer ramp
            } else {
                // Switch to INTAKE mode
                currentState = RobotState.INTAKE;
                deactivateFlywheel();  // Stop flywheel, lower ramp
            }
        }
        lastGP1_Y = gamepad1.y;
    }

    private void activateFlywheel() {
        launcher.setTargetRPM(flywheelRPM);  // Set target RPM for PIDF control
        launcher.setSpinning(true);
        launcher.setHoodPosition(hoodPosition);
        intakeTransfer.transferUp();
        flywheelOn = true;
    }

    private void deactivateFlywheel() {
        launcher.setSpinning(false);
        intakeTransfer.transferDown();
        flywheelOn = false;
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
            telemetry.addData("Flywheel", "OFF (GP1-Y to start)");
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
        telemetry.addLine("GP1: Y-ToggleMode B-Lock LB-EjectShoot | RT-In LT-Out | DPAD:U/D-Dist R-Far | Sticks-Drive");
        telemetry.addLine("GP2: A-Track X-CtrTur | LS-Turret DPAD:L-Tur U/D-RPM | LB/RB-Hood | START-ESTOP");

        telemetry.update();
    }

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
        turret.stop();
    }
}

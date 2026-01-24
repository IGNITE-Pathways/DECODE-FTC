package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.core.util.DistanceCalculator;

/**
 * *** COMPETITION TELEOP ***
 *
 * ALL SETTINGS (PIDF gains, RPM ranges, hood positions) are in RobotConstants.java!
 * After tuning, update values there - they automatically apply here.
 *
 * === GAMEPAD 1 (Driver/Shooter) ===
 * Sticks: Drive | RT: Intake | LT: Eject
 * A: Turret auto-track | Y: Flywheel toggle | B: Lock distance
 * DPAD UP: Enable distance detection | DPAD DOWN: Disable (use defaults)
 * DPAD LEFT: Far zone preset (5.7+ ft)
 *
 * === GAMEPAD 2 (Manual Adjustments) ===
 * Left Stick X: Manual turret | DPAD L/R: Fine turret adjust | X: Center turret
 * DPAD UP/DOWN: Adjust RPM (±100) | LB/RB: Adjust hood angle (±0.03)
 * B: Quick reset to defaults | START: EMERGENCY STOP
 *
 * === Shooting Workflow ===
 * 1. GP1-A: Enable turret tracking (aims at AprilTag)
 * 2. GP1-DPAD UP: Enable distance detection
 * 3. GP1-B: Lock distance when lined up
 * 4. GP1-Y: Spin up flywheel (uses locked distance)
 * 5. GP1-RT: Feed balls
 * 6. GP2-DPAD/BUMPERS: Fine-tune RPM/hood if needed
 */
public abstract class CompetitionTeleOpBase extends LinearOpMode {

    // === CONFIG ===
    private static final boolean SHOW_DEBUG_INFO = false;  // Set true to see limelight debug info

    protected AllianceColor alliance;

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private TurretLockOptimized turret;
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
    private boolean lastGP1_A = false;
    private boolean lastGP1_Y = false;
    private boolean lastGP1_B = false;
    private boolean lastGP1_DpadUp = false;
    private boolean lastGP1_DpadDown = false;
    private boolean lastGP1_DpadLeft = false;

    // Limelight distance tracking
    private double currentDistance = -1.0;  // Currently detected distance
    private double lockedDistance = -1.0;   // Locked distance for shooting

    // Gamepad 2 button states (manual turret & overrides)
    private boolean lastGP2_X = false;
    private boolean lastGP2_DpadLeft = false;
    private boolean lastGP2_DpadRight = false;
    private boolean lastGP2_DpadUp = false;
    private boolean lastGP2_DpadDown = false;
    private boolean lastGP2_B = false;
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
                handleTurret();               // GP1-A: Auto-tracking toggle
                handleManualTurretControl();  // GP2: Manual turret control
                handleIntake();               // GP1: RT/LT intake/eject
                handleDistanceLock();         // GP1: DPAD UP/DOWN/LEFT, B-lock
                handleFlywheelToggle();       // GP1-Y: Flywheel on/off
                handleManualOverrides();      // GP2: Manual RPM/hood adjustments
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

        turret = new TurretLockOptimized();
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

        // Reduce speed while flywheel is active for better shooting accuracy
        double speedMult = 1.0;
        if (flywheelOn) {
            speedMult = RobotConstants.SPEED_SHOOTING_MULTIPLIER;
        }

        driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < RobotConstants.JOYSTICK_DEAD_ZONE) {
            return 0;
        }
        return input;
    }

    // GP1-A: Toggle turret auto-tracking (limelight aims at AprilTag)
    private void handleTurret() {
        if (gamepad1.a && !lastGP1_A) {
            turretTracking = !turretTracking;
        }
        lastGP1_A = gamepad1.a;

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

        // Gamepad 2 DPAD Right: Small increment right
        if (gamepad2.dpad_right && !lastGP2_DpadRight) {
            currentPos += RobotConstants.TURRET_MANUAL_INCREMENT;
        }
        lastGP2_DpadRight = gamepad2.dpad_right;

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
        if (gamepad1.right_trigger > RobotConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > RobotConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
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

        // DPAD LEFT: Quick far zone preset (5.7+ ft, max distance)
        if (gamepad1.dpad_left && !lastGP1_DpadLeft) {
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
        lastGP1_DpadLeft = gamepad1.dpad_left;

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
        // All range values loaded from ShooterConstants - update there after testing!
        // Check ranges in order (most specific to least specific)

        if (distance >= RobotConstants.RANGE_1_MIN && distance < RobotConstants.RANGE_1_MAX) {
            flywheelRPM = RobotConstants.RANGE_1_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_1_HOOD_POSITION;
            selectedPreset = String.format("RANGE 1 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_2_MIN && distance < RobotConstants.RANGE_2_MAX) {
            flywheelRPM = RobotConstants.RANGE_2_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_2_HOOD_POSITION;
            selectedPreset = String.format("RANGE 2 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_3_MIN && distance < RobotConstants.RANGE_3_MAX) {
            flywheelRPM = RobotConstants.RANGE_3_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_3_HOOD_POSITION;
            selectedPreset = String.format("RANGE 3 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_4_MIN && distance < RobotConstants.RANGE_4_MAX) {
            flywheelRPM = RobotConstants.RANGE_4_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_4_HOOD_POSITION;
            selectedPreset = String.format("RANGE 4 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_6_MIN && distance < RobotConstants.RANGE_6_MAX) {
            // Prioritize Range 6 over Range 5 due to overlap
            flywheelRPM = RobotConstants.RANGE_6_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_6_HOOD_POSITION;
            selectedPreset = String.format("RANGE 6 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_5_MIN && distance < RobotConstants.RANGE_5_MAX) {
            flywheelRPM = RobotConstants.RANGE_5_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_5_HOOD_POSITION;
            selectedPreset = String.format("RANGE 5 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_7_MIN && distance < RobotConstants.RANGE_7_MAX) {
            flywheelRPM = RobotConstants.RANGE_7_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_7_HOOD_POSITION;
            selectedPreset = String.format("RANGE 7 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_FAR_MIN) {
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
            selectedPreset = String.format("FAR ZONE (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else {
            // Below minimum range - use defaults
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
            selectedPreset = "OUT OF RANGE - DEFAULT";
        }
    }

    // GP1-Y: Toggle flywheel ON/OFF (uses locked distance preset)
    private void handleFlywheelToggle() {
        if (gamepad1.y && !lastGP1_Y) {
            if (!flywheelOn) {
                activateFlywheel();  // Spin up to target RPM, raise transfer ramp
            } else {
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
    // - DPAD UP/DOWN: Fine-tune RPM (±100 RPM per press)
    // - LEFT/RIGHT BUMPER: Adjust hood angle (±0.03 per press)
    // - B: Quick reset to defaults
    private void handleManualOverrides() {
        // B: Quick reset - return to safe defaults
        if (gamepad2.b && !lastGP2_B) {
            if (flywheelOn) {
                deactivateFlywheel();
            }
            distanceDetectionEnabled = false;
            currentDistance = -1.0;
            lockedDistance = -1.0;
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
            selectedPreset = "DEFAULT";
            turretTracking = false;
            launcher.setHoodPosition(hoodPosition);
            gamepad2.rumble(200);
        }
        lastGP2_B = gamepad2.b;

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

        // === EMERGENCY STOP WARNING ===
        if (emergencyStopped) {
            telemetry.addLine("*** E-STOP ACTIVE - Release GP2-START ***");
        }

        // === SHOOTING STATUS ===
        if (flywheelOn) {
            double currentRPM = launcher.getCurrentRPM();
            double error = Math.abs(flywheelRPM - currentRPM);
            boolean ready = error < 50;  // Ready if within 50 RPM

            String status = ready ? ">>> READY TO SHOOT <<<" : "SPINNING UP...";
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
        telemetry.addLine("GP1: A-Track Y-Shoot B-Lock | DPAD:U/D-Dist L-Far | RT-In LT-Out");
        telemetry.addLine("GP2: LS-Turret DPAD:L/R-Tur U/D-RPM LB/RB-Hood X-Ctr B-Reset | START-ESTOP");

        telemetry.update();
    }

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
    }
}

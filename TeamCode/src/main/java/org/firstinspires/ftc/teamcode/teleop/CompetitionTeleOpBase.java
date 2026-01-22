package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.core.util.DistanceCalculator;

/**
 * Base class for Competition TeleOp - contains all common logic.
 * Blue and Red versions extend this and just set the alliance color.
 *
 * GAMEPAD 1 (Driver + Shooter):
 * - Left Stick: Drive forward/strafe
 * - Right Stick X: Rotate
 * - A: Toggle turret auto-tracking ON/OFF
 * - Y: Toggle flywheel ON/OFF
 * - B: Lock current distance (calculates from limelight)
 * - RT: Intake
 * - LT: Eject
 *
 * GAMEPAD 2 (Manual Turret Control):
 * - Left Stick X: Manual turret control (continuous)
 * - DPAD Left: Move turret left (small increment)
 * - DPAD Right: Move turret right (small increment)
 * - X: Reset turret to center position
 *
 * DISTANCE LOCKING WORKFLOW:
 * 1. Point robot at AprilTag (use A to toggle auto-tracking)
 * 2. Press B to lock current distance (calculates once, stores it)
 * 3. Press Y to activate flywheel (uses locked distance to set power & hood)
 * 4. Flywheel spins at preset power/hood for that distance range
 *
 * DEFAULT (no distance locked): 80% power, 0.25 hood (lowest)
 */
public abstract class CompetitionTeleOpBase extends LinearOpMode {

    protected AllianceColor alliance;

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private TurretLockOptimized turret;
    private Limelight3A limelight;

    // State
    private boolean flywheelOn = false;
    private double flywheelPower = TeleOpConstants.DEFAULT_FLYWHEEL_POWER;
    private double hoodPosition = TeleOpConstants.DEFAULT_HOOD_POSITION;
    private String selectedPreset = "DEFAULT";
    private boolean turretTracking = false;

    // Gamepad 1 button states
    private boolean lastGP1_A = false;
    private boolean lastGP1_Y = false;
    private boolean lastGP1_B = false;

    // Gamepad 2 button states (manual turret)
    private boolean lastGP2_X = false;
    private boolean lastGP2_DpadLeft = false;
    private boolean lastGP2_DpadRight = false;

    // Manual turret control
    private static final double TURRET_INCREMENT = 0.02;  // Small increment for DPAD
    private static final double TURRET_MIN = 0.0;
    private static final double TURRET_MAX = 1.0;
    private static final double TURRET_CENTER = 0.5;

    // Limelight distance tracking
    private double lockedDistance = -1.0;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            handleDriving();
            handleTurret();
            handleManualTurretControl();
            handleIntake();
            handleDistanceLock();
            handleFlywheelToggle();
            launcher.update();
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

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

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
            speedMult = TeleOpConstants.SPEED_SHOOTING_MULTIPLIER;
        }

        driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < TeleOpConstants.JOYSTICK_DEAD_ZONE) {
            return 0;
        }
        return input;
    }

    private void handleTurret() {
        // Gamepad 1 A: Toggle auto-tracking
        if (gamepad1.a && !lastGP1_A) {
            turretTracking = !turretTracking;
        }
        lastGP1_A = gamepad1.a;

        // Update turret if auto-tracking is enabled
        if (turretTracking) {
            turret.update();
        }
        // If auto-tracking is off, manual control from gamepad2 will handle it
    }

    private void handleManualTurretControl() {
        // Only allow manual control when auto-tracking is OFF
        if (turretTracking) {
            return;
        }

        double currentPos = turret.getServoPosition();

        // Gamepad 2 Left Stick X: Continuous manual control
        double stickInput = -gamepad2.left_stick_x;  // Inverted for intuitive control
        if (Math.abs(stickInput) > 0.1) {
            double adjustment = stickInput * 0.01;  // Scale down for smooth control
            currentPos += adjustment;
        }

        // Gamepad 2 DPAD Left: Small increment left
        if (gamepad2.dpad_left && !lastGP2_DpadLeft) {
            currentPos -= TURRET_INCREMENT;
        }
        lastGP2_DpadLeft = gamepad2.dpad_left;

        // Gamepad 2 DPAD Right: Small increment right
        if (gamepad2.dpad_right && !lastGP2_DpadRight) {
            currentPos += TURRET_INCREMENT;
        }
        lastGP2_DpadRight = gamepad2.dpad_right;

        // Gamepad 2 X: Reset to center
        if (gamepad2.x && !lastGP2_X) {
            currentPos = TURRET_CENTER;
        }
        lastGP2_X = gamepad2.x;

        // Clamp position to valid range
        currentPos = Math.max(TURRET_MIN, Math.min(TURRET_MAX, currentPos));

        // Apply position
        turret.setPositionDirect(currentPos);
    }

    private void handleIntake() {
        if (gamepad1.right_trigger > TeleOpConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > TeleOpConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
        }
    }

    // ========================================
    // GAMEPAD 2: LAUNCHER CONTROLS
    // ========================================

    private void handleDistanceLock() {
        // Gamepad 1 B: Lock distance
        if (gamepad1.b && !lastGP1_B) {
            // Only calculate distance when B button is pressed (performance optimization)
            double distance = calculateCurrentDistance();
            if (distance > 0) {
                lockedDistance = distance;
                applyDistancePreset(lockedDistance);
                launcher.setHoodPosition(hoodPosition);
            }
        }
        lastGP1_B = gamepad1.b;
    }

    private double calculateCurrentDistance() {
        if (limelight == null || !limelight.isConnected()) {
            return -1.0;
        }

        LLResult result = limelight.getLatestResult();
        return DistanceCalculator.calculateDistance(result);
    }

    private void applyDistancePreset(double distance) {
        // Check ranges in order (most specific to least specific)
        // Note: Some ranges overlap, prioritize based on testing results

        if (distance >= TeleOpConstants.RANGE_1_MIN && distance < TeleOpConstants.RANGE_1_MAX) {
            // 2.47 - 2.84 ft
            flywheelPower = TeleOpConstants.RANGE_1_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_1_HOOD_POSITION;
            selectedPreset = String.format("RANGE 1 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_2_MIN && distance < TeleOpConstants.RANGE_2_MAX) {
            // 2.84 - 3.2 ft
            flywheelPower = TeleOpConstants.RANGE_2_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_2_HOOD_POSITION;
            selectedPreset = String.format("RANGE 2 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_3_MIN && distance < TeleOpConstants.RANGE_3_MAX) {
            // 3.21 - 4.0 ft
            flywheelPower = TeleOpConstants.RANGE_3_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_3_HOOD_POSITION;
            selectedPreset = String.format("RANGE 3 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_4_MIN && distance < TeleOpConstants.RANGE_4_MAX) {
            // 4.0 - 4.5 ft
            flywheelPower = TeleOpConstants.RANGE_4_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_4_HOOD_POSITION;
            selectedPreset = String.format("RANGE 4 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_6_MIN && distance < TeleOpConstants.RANGE_6_MAX) {
            // 4.84 - 5.25 ft (prioritize this over Range 5 due to overlap)
            flywheelPower = TeleOpConstants.RANGE_6_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_6_HOOD_POSITION;
            selectedPreset = String.format("RANGE 6 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_5_MIN && distance < TeleOpConstants.RANGE_5_MAX) {
            // 4.6 - 5.0 ft
            flywheelPower = TeleOpConstants.RANGE_5_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_5_HOOD_POSITION;
            selectedPreset = String.format("RANGE 5 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_7_MIN && distance < TeleOpConstants.RANGE_7_MAX) {
            // 5.2 - 5.7 ft
            flywheelPower = TeleOpConstants.RANGE_7_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_7_HOOD_POSITION;
            selectedPreset = String.format("RANGE 7 (%.2fft)", distance);

        } else if (distance >= TeleOpConstants.RANGE_FAR_MIN) {
            // Far shooting zone (> 5.7 ft)
            flywheelPower = TeleOpConstants.RANGE_FAR_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.RANGE_FAR_HOOD_POSITION;
            selectedPreset = String.format("FAR ZONE (%.2fft)", distance);

        } else {
            // Below minimum range
            flywheelPower = TeleOpConstants.DEFAULT_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.DEFAULT_HOOD_POSITION;
            selectedPreset = "OUT OF RANGE - DEFAULT";
        }
    }

    private void handleFlywheelToggle() {
        // Gamepad 1 Y: Toggle flywheel
        if (gamepad1.y && !lastGP1_Y) {
            if (!flywheelOn) {
                activateFlywheel();
            } else {
                deactivateFlywheel();
            }
        }
        lastGP1_Y = gamepad1.y;
    }

    private void activateFlywheel() {
        launcher.setPower(flywheelPower);
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

    // ========================================
    // TELEMETRY & SHUTDOWN
    // ========================================

    private void updateTelemetry() {
        telemetry.addLine("=== " + alliance.name() + " TELEOP ===");
        telemetry.addLine();

        // Distance & Preset Info
        telemetry.addLine("--- SHOOTING ---");
        telemetry.addData("Locked Distance", lockedDistance > 0 ? String.format("%.2f ft", lockedDistance) : "NONE (GP1-B)");
        telemetry.addData("Preset", selectedPreset);
        telemetry.addLine();

        // Shooter Status
        telemetry.addData("Flywheel", flywheelOn ? String.format("ON %d%%", (int)(flywheelPower*100)) : "OFF (GP1-Y)");
        telemetry.addData("Hood Position", "%.2f", hoodPosition);
        telemetry.addData("Transfer Ramp", intakeTransfer.isTransferUp() ? "UP" : "DOWN");
        telemetry.addLine();

        // Turret Status
        String turretStatus = turretTracking ? (turret.isLocked() ? "AUTO-LOCKED" : "AUTO-TRACKING") : "MANUAL";
        telemetry.addData("Turret Mode", turretStatus + " (GP1-A)");
        telemetry.addData("Turret Position", "%.2f", turret.getServoPosition());
        telemetry.addLine();

        // Controls Reminder
        telemetry.addLine("--- GAMEPAD 1 (Driver/Shooter) ---");
        telemetry.addLine("Sticks: Drive | A: Turret auto");
        telemetry.addLine("Y: Flywheel | B: Lock distance");
        telemetry.addLine("RT: Intake | LT: Eject");
        telemetry.addLine();
        telemetry.addLine("--- GAMEPAD 2 (Manual Turret) ---");
        telemetry.addLine("L-Stick X: Move turret");
        telemetry.addLine("DPAD L/R: Fine adjust | X: Center");

        telemetry.update();
    }

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
    }
}

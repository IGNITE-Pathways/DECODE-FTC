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

    // Button states for toggle
    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastB = false;

    // Limelight distance tracking
    private double lockedDistance = -1.0;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            handleDriving();
            handleTurret();
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
        if (gamepad1.a && !lastA) {
            turretTracking = !turretTracking;
        }
        lastA = gamepad1.a;

        if (turretTracking) {
            turret.update();
        } else {
            turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
        }
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
        if (gamepad2.b && !lastB) {
            // Only calculate distance when B button is pressed (performance optimization)
            double distance = calculateCurrentDistance();
            if (distance > 0) {
                lockedDistance = distance;
                applyDistancePreset(lockedDistance);
                launcher.setHoodPosition(hoodPosition);
            }
        }
        lastB = gamepad2.b;
    }

    private double calculateCurrentDistance() {
        if (limelight == null || !limelight.isConnected()) {
            return -1.0;
        }

        LLResult result = limelight.getLatestResult();
        return DistanceCalculator.calculateDistance(result);
    }

    private void applyDistancePreset(double distance) {
        if (distance >= TeleOpConstants.DISTANCE_CLOSE_MIN && distance < TeleOpConstants.DISTANCE_CLOSE_MAX) {
            flywheelPower = TeleOpConstants.CLOSE_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.CLOSE_HOOD_POSITION;
            selectedPreset = "AUTO CLOSE";
        } else if (distance >= TeleOpConstants.DISTANCE_MID_MIN && distance < TeleOpConstants.DISTANCE_MID_MAX) {
            flywheelPower = TeleOpConstants.MID_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.MID_HOOD_POSITION;
            selectedPreset = "AUTO MID";
        } else if (distance >= TeleOpConstants.DISTANCE_FAR_MIN && distance <= TeleOpConstants.DISTANCE_FAR_MAX) {
            flywheelPower = TeleOpConstants.FAR_FLYWHEEL_POWER;
            hoodPosition = TeleOpConstants.FAR_HOOD_POSITION;
            selectedPreset = "AUTO FAR";
        } else {
            selectedPreset = "OUT OF RANGE";
        }
    }

    private void handleFlywheelToggle() {
        if (gamepad2.y && !lastY) {
            if (!flywheelOn) {
                activateFlywheel();
            } else {
                deactivateFlywheel();
            }
        }
        lastY = gamepad2.y;
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
        telemetry.addData("Preset", selectedPreset);
        telemetry.addData("Flywheel", flywheelOn ? "ON " + (int)(flywheelPower*100) + "%" : "OFF");
        telemetry.addData("Hood", "%.2f", hoodPosition);
        telemetry.addData("Transfer Ramp", intakeTransfer.isTransferUp() ? "UP" : "DOWN");
        telemetry.addData("Ramp Position", "%.2f", intakeTransfer.getTransferPosition());
        telemetry.addData("Turret", turretTracking ? (turret.isLocked() ? "LOCKED ON" : "TRACKING") : "MANUAL");
        telemetry.addLine();
        telemetry.addData("Locked Distance", lockedDistance > 0 ? String.format("%.2f ft", lockedDistance) : "NOT SET");
        telemetry.update();
    }

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
    }
}

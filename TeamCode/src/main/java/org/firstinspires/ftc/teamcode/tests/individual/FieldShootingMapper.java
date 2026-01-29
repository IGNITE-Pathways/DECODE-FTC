package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.LimelightLocalization;
import org.firstinspires.ftc.teamcode.core.components.ActualTurretLockOn;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * *** FIELD SHOOTING POSITION MAPPER ***
 *
 * This OpMode allows you to drive around the field and record optimal shooting settings
 * for different positions. Use this to build a position-based shooting lookup table.
 *
 * HOW TO USE:
 * ===========
 * 1. Drive to a position in a launch zone
 * 2. Adjust flywheel RPM and hood angle until shots are accurate
 * 3. Press A to SAVE current position + settings
 * 4. Repeat for multiple positions
 * 5. At the end, all saved positions will be logged to telemetry
 * 6. Copy the output to create a lookup table in code
 *
 * CONTROLS:
 * =========
 * GAMEPAD 1 (DRIVER):
 * - Left Stick: Drive forward/strafe
 * - Right Stick X: Rotate
 * - Y: Toggle flywheel ON/OFF
 * - A: SAVE current position + shooter settings
 * - B: Clear all saved positions
 * - X: Delete last saved position
 *
 * GAMEPAD 2 (SHOOTER):
 * - DPAD UP: Increase RPM (+50)
 * - DPAD DOWN: Decrease RPM (-50)
 * - DPAD LEFT: Decrease hood angle (-0.02)
 * - DPAD RIGHT: Increase hood angle (+0.02)
 * - LB: Fine decrease hood (-0.005)
 * - RB: Fine increase hood (+0.005)
 * - A: Enable turret auto-lock
 * - X: Disable turret auto-lock (manual center)
 * - RT: Test shoot (run intake)
 *
 * WORKFLOW:
 * =========
 * 1. Position robot at first shooting location
 * 2. Toggle flywheel ON (GP1-Y)
 * 3. Adjust RPM/hood until shots are perfect (GP2)
 * 4. Press GP1-A to save this position
 * 5. Move to next location and repeat
 * 6. When done, review saved positions in telemetry
 */
@TeleOp(name = "Mapper: Field Shooting Positions", group = "Mapping")
public class FieldShootingMapper extends LinearOpMode {

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private LimelightLocalization localization;
    private ActualTurretLockOn turret;
    private IntakeTransfer intakeTransfer;

    // Shooting state
    private boolean flywheelOn = false;
    private double flywheelRPM = 2400;
    private double hoodAngle = 0.55;
    private boolean turretLockEnabled = false;

    // Position data storage
    private List<ShootingPosition> savedPositions = new ArrayList<>();
    private int positionCounter = 0;

    // Button states - GP1
    private boolean lastGP1_Y = false;
    private boolean lastGP1_A = false;
    private boolean lastGP1_B = false;
    private boolean lastGP1_X = false;

    // Button states - GP2
    private boolean lastGP2_DpadUp = false;
    private boolean lastGP2_DpadDown = false;
    private boolean lastGP2_DpadLeft = false;
    private boolean lastGP2_DpadRight = false;
    private boolean lastGP2_LB = false;
    private boolean lastGP2_RB = false;
    private boolean lastGP2_A = false;
    private boolean lastGP2_X = false;

    // Feedback
    private String lastAction = "Ready to map";
    private ElapsedTime actionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeComponents();

        telemetry.addLine("=== FIELD SHOOTING MAPPER ===");
        telemetry.addLine();
        telemetry.addLine("Drive to positions and save shooting settings");
        telemetry.addLine("GP1-A: Save position | GP1-Y: Toggle flywheel");
        telemetry.addLine("GP2: Adjust RPM/hood | GP2-RT: Test shoot");
        telemetry.addLine();
        telemetry.addLine("Press START when ready...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update components
            localization.update();
            launcher.update();
            if (turretLockEnabled) {
                turret.update();
            }

            // Handle controls
            handleDriving();
            handleShooterAdjustments();
            handlePositionMapping();
            handleIntake();

            // Display telemetry
            displayTelemetry();
        }

        // Cleanup
        shutdown();
    }

    // ========================================
    // INITIALIZATION
    // ========================================

    private void initializeComponents() {
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        localization = new LimelightLocalization();
        localization.initialize(hardwareMap, telemetry);

        turret = new ActualTurretLockOn();
        turret.initialize(hardwareMap, telemetry, AllianceColor.BLUE);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        intakeTransfer.transferUp();
    }

    // ========================================
    // DRIVING
    // ========================================

    private void handleDriving() {
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Apply deadzone
        fwd = applyDeadzone(fwd);
        str = applyDeadzone(str);
        rot = applyDeadzone(rot);

        // Slower speed for precise positioning
        double speedMult = 0.5;

        driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.1 ? 0 : value;
    }

    // ========================================
    // SHOOTER ADJUSTMENTS
    // ========================================

    private void handleShooterAdjustments() {
        // GP1-Y: Toggle flywheel
        if (gamepad1.y && !lastGP1_Y) {
            flywheelOn = !flywheelOn;
            launcher.setSpinning(flywheelOn);
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
                launcher.setHoodPosition(hoodAngle);
                intakeTransfer.transferUp();
                lastAction = "Flywheel ON";
            } else {
                intakeTransfer.transferDown();
                lastAction = "Flywheel OFF";
            }
            actionTimer.reset();
        }
        lastGP1_Y = gamepad1.y;

        // GP2: RPM adjustments
        if (gamepad2.dpad_up && !lastGP2_DpadUp) {
            flywheelRPM += 50;
            if (flywheelOn) launcher.setTargetRPM(flywheelRPM);
            lastAction = String.format("RPM: %.0f", flywheelRPM);
            actionTimer.reset();
        }
        lastGP2_DpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastGP2_DpadDown) {
            flywheelRPM = Math.max(1000, flywheelRPM - 50);
            if (flywheelOn) launcher.setTargetRPM(flywheelRPM);
            lastAction = String.format("RPM: %.0f", flywheelRPM);
            actionTimer.reset();
        }
        lastGP2_DpadDown = gamepad2.dpad_down;

        // GP2: Hood adjustments
        if (gamepad2.dpad_right && !lastGP2_DpadRight) {
            hoodAngle = Math.min(0.9, hoodAngle + 0.02);
            if (flywheelOn) launcher.setHoodPosition(hoodAngle);
            lastAction = String.format("Hood: %.3f", hoodAngle);
            actionTimer.reset();
        }
        lastGP2_DpadRight = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !lastGP2_DpadLeft) {
            hoodAngle = Math.max(0.1, hoodAngle - 0.02);
            if (flywheelOn) launcher.setHoodPosition(hoodAngle);
            lastAction = String.format("Hood: %.3f", hoodAngle);
            actionTimer.reset();
        }
        lastGP2_DpadLeft = gamepad2.dpad_left;

        // GP2: Fine hood adjustments
        if (gamepad2.right_bumper && !lastGP2_RB) {
            hoodAngle = Math.min(0.9, hoodAngle + 0.005);
            if (flywheelOn) launcher.setHoodPosition(hoodAngle);
            lastAction = String.format("Hood: %.3f (fine)", hoodAngle);
            actionTimer.reset();
        }
        lastGP2_RB = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !lastGP2_LB) {
            hoodAngle = Math.max(0.1, hoodAngle - 0.005);
            if (flywheelOn) launcher.setHoodPosition(hoodAngle);
            lastAction = String.format("Hood: %.3f (fine)", hoodAngle);
            actionTimer.reset();
        }
        lastGP2_LB = gamepad2.left_bumper;

        // GP2-A: Enable turret auto-lock
        if (gamepad2.a && !lastGP2_A) {
            turretLockEnabled = true;
            lastAction = "Turret auto-lock ON";
            actionTimer.reset();
        }
        lastGP2_A = gamepad2.a;

        // GP2-X: Disable turret, center
        if (gamepad2.x && !lastGP2_X) {
            turretLockEnabled = false;
            turret.setPositionDirect(0.5);
            lastAction = "Turret centered";
            actionTimer.reset();
        }
        lastGP2_X = gamepad2.x;
    }

    // ========================================
    // INTAKE/SHOOTING
    // ========================================

    private void handleIntake() {
        // GP2-RT: Test shoot
        if (gamepad2.right_trigger > 0.1) {
            intakeTransfer.startIntake(gamepad2.right_trigger);
        } else {
            intakeTransfer.stopIntake();
        }
    }

    // ========================================
    // POSITION MAPPING
    // ========================================

    private void handlePositionMapping() {
        // GP1-A: Save current position
        if (gamepad1.a && !lastGP1_A) {
            saveCurrentPosition();
        }
        lastGP1_A = gamepad1.a;

        // GP1-B: Clear all saved positions
        if (gamepad1.b && !lastGP1_B) {
            savedPositions.clear();
            positionCounter = 0;
            lastAction = "All positions cleared!";
            actionTimer.reset();
        }
        lastGP1_B = gamepad1.b;

        // GP1-X: Delete last saved position
        if (gamepad1.x && !lastGP1_X) {
            if (!savedPositions.isEmpty()) {
                savedPositions.remove(savedPositions.size() - 1);
                lastAction = String.format("Deleted position #%d", positionCounter);
                positionCounter--;
                actionTimer.reset();
            }
        }
        lastGP1_X = gamepad1.x;
    }

    private void saveCurrentPosition() {
        positionCounter++;

        ShootingPosition pos = new ShootingPosition();
        pos.id = positionCounter;
        pos.x = localization.getX();
        pos.y = localization.getY();
        pos.heading = localization.getHeading();
        pos.flywheelRPM = flywheelRPM;
        pos.hoodAngle = hoodAngle;
        pos.turretPosition = turret.getServoPosition();
        pos.distanceToTarget = localization.getDistance();

        savedPositions.add(pos);

        lastAction = String.format("SAVED #%d @ (%.1f, %.1f)", positionCounter, pos.x, pos.y);
        actionTimer.reset();

        // Log to telemetry for copy-paste
        telemetry.log().add("=== POSITION #" + positionCounter + " ===");
        telemetry.log().add(String.format("X: %.2f in, Y: %.2f in, Heading: %.1f°", pos.x, pos.y, pos.heading));
        telemetry.log().add(String.format("RPM: %.0f, Hood: %.3f, Turret: %.3f", pos.flywheelRPM, pos.hoodAngle, pos.turretPosition));
        telemetry.log().add(String.format("Distance to target: %.1f in", pos.distanceToTarget));
        telemetry.log().add("");
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void displayTelemetry() {
        telemetry.addLine("=== FIELD SHOOTING MAPPER ===");
        telemetry.addLine();

        // Current position
        telemetry.addLine("--- ROBOT POSITION ---");
        telemetry.addData("X", "%.2f in", localization.getX());
        telemetry.addData("Y", "%.2f in", localization.getY());
        telemetry.addData("Heading", "%.1f°", localization.getHeading());
        telemetry.addData("Distance to Target", "%.1f in", localization.getDistance());
        telemetry.addLine();

        // Current shooter settings
        telemetry.addLine("--- CURRENT SETTINGS ---");
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        if (flywheelOn) {
            telemetry.addData("Target RPM", "%.0f", flywheelRPM);
            telemetry.addData("Actual RPM", "%.0f", launcher.getCurrentRPM());
        }
        telemetry.addData("Hood Angle", "%.3f", hoodAngle);
        telemetry.addData("Turret", turretLockEnabled ? "AUTO-LOCK" : "MANUAL");
        if (turretLockEnabled) {
            telemetry.addData("Turret Locked", turret.isLocked() ? "YES" : "no");
        }
        telemetry.addLine();

        // Saved positions
        telemetry.addLine("--- SAVED POSITIONS ---");
        telemetry.addData("Count", savedPositions.size());
        if (!savedPositions.isEmpty()) {
            ShootingPosition last = savedPositions.get(savedPositions.size() - 1);
            telemetry.addData("Last Saved", String.format("#%d @ (%.1f, %.1f)", last.id, last.x, last.y));
        }
        telemetry.addLine();

        // Last action feedback
        if (actionTimer.seconds() < 2.0) {
            telemetry.addLine(">>> " + lastAction + " <<<");
        }
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("GP1: A-Save B-Clear X-Undo Y-Flywheel");
        telemetry.addLine("GP2: DPAD-Adjust A-Lock X-Center RT-Shoot");

        telemetry.update();
    }

    // ========================================
    // SHUTDOWN
    // ========================================

    private void shutdown() {
        launcher.setSpinning(false);
        turret.stop();
        intakeTransfer.transferDown();
        intakeTransfer.stopIntake();

        // Final output of all saved positions
        telemetry.log().add("======================");
        telemetry.log().add("FINAL POSITION DATA");
        telemetry.log().add("======================");
        for (ShootingPosition pos : savedPositions) {
            telemetry.log().add(String.format("Position #%d:", pos.id));
            telemetry.log().add(String.format("  Location: (%.2f, %.2f) @ %.1f°", pos.x, pos.y, pos.heading));
            telemetry.log().add(String.format("  Settings: RPM=%.0f, Hood=%.3f, Turret=%.3f",
                pos.flywheelRPM, pos.hoodAngle, pos.turretPosition));
            telemetry.log().add(String.format("  Distance: %.1f in", pos.distanceToTarget));
            telemetry.log().add("");
        }
        telemetry.log().add("======================");
        telemetry.log().add("Copy this data to create lookup table");
    }

    // ========================================
    // DATA STRUCTURE
    // ========================================

    private static class ShootingPosition {
        int id;
        double x;              // inches
        double y;              // inches
        double heading;        // degrees
        double flywheelRPM;
        double hoodAngle;
        double turretPosition;
        double distanceToTarget;  // inches
    }
}

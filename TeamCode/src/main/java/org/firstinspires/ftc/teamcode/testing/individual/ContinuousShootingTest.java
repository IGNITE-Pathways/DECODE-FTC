package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.turret.ActualTurretLockOn;
import org.firstinspires.ftc.teamcode.testing.individual.LimelightLocalization;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.testing.ContinuousShootingCalculator;
import org.firstinspires.ftc.teamcode.testing.ContinuousShootingCalculator.ShootingParameters;

/**
 * *** CONTINUOUS SHOOTING EQUATION TEST ***
 *
 * Tests the linear equation-based shooting system.
 * Uses Limelight distance detection to continuously calculate optimal RPM and hood angle.
 *
 * EQUATIONS BEING TESTED:
 * RPM(x) = 2950 + 160(x - 6.5)
 * Hood(x) = 0.72 + 0.008(x - 6.5)
 *
 * HOW TO USE:
 * ===========
 * 1. Position robot at various distances from target
 * 2. Toggle flywheel ON (GP1-Y)
 * 3. System automatically calculates RPM and hood from Limelight distance
 * 4. Enable turret auto-lock (GP2-A) to aim at AprilTag
 * 5. Test shoot (GP1-RT) and observe accuracy
 * 6. Record results for each distance
 * 7. Adjust equation parameters if needed
 *
 * CONTROLS:
 * =========
 * GAMEPAD 1 (Driver/Shooter):
 * - Left Stick: Drive
 * - Right Stick X: Rotate
 * - Y: Toggle flywheel ON/OFF (uses continuous equations)
 * - RT: Test shoot (run intake)
 * - LT: Eject
 * - A: Print lookup table to telemetry log
 * - B: Print equation comparison to telemetry log
 *
 * GAMEPAD 2 (Fine Adjustments):
 * - A: Enable turret auto-lock
 * - X: Disable turret, center
 * - DPAD UP: Manual RPM +50 (override equation)
 * - DPAD DOWN: Manual RPM -50 (override equation)
 * - DPAD RIGHT: Manual hood +0.01 (override equation)
 * - DPAD LEFT: Manual hood -0.01 (override equation)
 * - B: Reset to equation-based (clear overrides)
 *
 * TESTING WORKFLOW:
 * =================
 * 1. Drive to known distance (e.g., 3 ft, 5 ft, 7 ft, 10 ft)
 * 2. Press GP1-Y to activate flywheel (auto-calculates settings)
 * 3. Press GP2-A to lock turret on AprilTag
 * 4. Wait for flywheel to reach target RPM
 * 5. Press GP1-RT to test shoot
 * 6. Record: Distance, Calculated RPM/Hood, Shot Result (hit/miss/high/low)
 * 7. If needed, use GP2 DPAD to manually adjust and find optimal values
 * 8. Compare optimal values to equation predictions
 * 9. Adjust equation parameters in ContinuousShootingCalculator.java if needed
 */
@TeleOp(name = "Test: Continuous Shooting Equations", group = "Testing")
public class ContinuousShootingTest extends LinearOpMode {

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private ActualTurretLockOn turret;
    private LimelightLocalization localization;

    // Shooting state
    private boolean flywheelOn = false;
    private boolean turretLockEnabled = false;
    private boolean useEquations = true;  // Use equations vs manual override

    // Calculated values
    private double calculatedRPM = 0;
    private double calculatedHood = 0;
    private double currentDistance = 0;

    // Manual overrides
    private double manualRPM = 0;
    private double manualHood = 0;

    // Button states - GP1
    private boolean lastGP1_Y = false;
    private boolean lastGP1_A = false;
    private boolean lastGP1_B = false;

    // Button states - GP2
    private boolean lastGP2_A = false;
    private boolean lastGP2_X = false;
    private boolean lastGP2_B = false;
    private boolean lastGP2_DpadUp = false;
    private boolean lastGP2_DpadDown = false;
    private boolean lastGP2_DpadLeft = false;
    private boolean lastGP2_DpadRight = false;

    @Override
    public void runOpMode() {
        initializeComponents();

        // Print equation info at startup
        telemetry.addLine("=== CONTINUOUS SHOOTING TEST ===");
        telemetry.addLine();
        telemetry.addLine("EQUATIONS:");
        telemetry.addData("RPM", ContinuousShootingCalculator.getRPMEquation());
        telemetry.addData("Hood", ContinuousShootingCalculator.getHoodEquation());
        telemetry.addLine();
        telemetry.addLine("GP1-A: Print lookup table");
        telemetry.addLine("GP1-B: Print equation comparison");
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

            // Get current distance from Limelight
            currentDistance = localization.getDistance() / 12.0;  // Convert inches to feet

            // Calculate shooting parameters using equations
            if (useEquations && currentDistance > 0) {
                ShootingParameters params = ContinuousShootingCalculator.calculate(currentDistance);
                calculatedRPM = params.rpm;
                calculatedHood = params.hoodAngle;
            }

            // Handle controls
            handleDriving();
            handleShooterControls();
            handleTurretControls();
            handleManualAdjustments();
            handleIntake();
            handleDebugOutput();

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

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        localization = new LimelightLocalization();
        localization.initialize(hardwareMap, telemetry);

        turret = new ActualTurretLockOn();
        turret.initialize(hardwareMap, telemetry, AllianceColor.BLUE);
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

        // Moderate speed for positioning
        double speedMult = 0.6;

        driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.1 ? 0 : value;
    }

    // ========================================
    // SHOOTER CONTROLS
    // ========================================

    private void handleShooterControls() {
        // GP1-Y: Toggle flywheel with continuous equations
        if (gamepad1.y && !lastGP1_Y) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                // Calculate settings from current distance
                if (currentDistance > 0) {
                    ShootingParameters params = ContinuousShootingCalculator.calculate(currentDistance);

                    if (useEquations) {
                        launcher.setTargetRPM(params.rpm);
                        launcher.setHoodPosition(params.hoodAngle);
                    } else {
                        // Use manual overrides
                        launcher.setTargetRPM(manualRPM);
                        launcher.setHoodPosition(manualHood);
                    }
                } else {
                    // No distance detected, use baseline
                    launcher.setTargetRPM(2950);
                    launcher.setHoodPosition(0.72);
                }

                launcher.setSpinning(true);
                intakeTransfer.transferUp();
            } else {
                launcher.setSpinning(false);
                intakeTransfer.transferDown();
            }
        }
        lastGP1_Y = gamepad1.y;

        // Continuously update settings while flywheel is on (if using equations)
        if (flywheelOn && useEquations && currentDistance > 0) {
            launcher.setTargetRPM(calculatedRPM);
            launcher.setHoodPosition(calculatedHood);
        }
    }

    // ========================================
    // TURRET CONTROLS
    // ========================================

    private void handleTurretControls() {
        // GP2-A: Enable turret auto-lock
        if (gamepad2.a && !lastGP2_A) {
            turretLockEnabled = true;
        }
        lastGP2_A = gamepad2.a;

        // GP2-X: Disable turret, center
        if (gamepad2.x && !lastGP2_X) {
            turretLockEnabled = false;
            turret.setPositionDirect(0.5);
        }
        lastGP2_X = gamepad2.x;
    }

    // ========================================
    // MANUAL ADJUSTMENTS
    // ========================================

    private void handleManualAdjustments() {
        // GP2-B: Reset to equation-based mode
        if (gamepad2.b && !lastGP2_B) {
            useEquations = true;
        }
        lastGP2_B = gamepad2.b;

        // GP2 DPAD: Manual overrides
        if (gamepad2.dpad_up && !lastGP2_DpadUp) {
            useEquations = false;
            manualRPM = (useEquations ? calculatedRPM : manualRPM) + 50;
            if (flywheelOn) launcher.setTargetRPM(manualRPM);
        }
        lastGP2_DpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastGP2_DpadDown) {
            useEquations = false;
            manualRPM = Math.max(1000, (useEquations ? calculatedRPM : manualRPM) - 50);
            if (flywheelOn) launcher.setTargetRPM(manualRPM);
        }
        lastGP2_DpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_right && !lastGP2_DpadRight) {
            useEquations = false;
            manualHood = Math.min(0.9, (useEquations ? calculatedHood : manualHood) + 0.01);
            if (flywheelOn) launcher.setHoodPosition(manualHood);
        }
        lastGP2_DpadRight = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !lastGP2_DpadLeft) {
            useEquations = false;
            manualHood = Math.max(0.1, (useEquations ? calculatedHood : manualHood) - 0.01);
            if (flywheelOn) launcher.setHoodPosition(manualHood);
        }
        lastGP2_DpadLeft = gamepad2.dpad_left;

        // Initialize manual values from equations on first override
        if (!useEquations && manualRPM == 0) {
            manualRPM = calculatedRPM;
            manualHood = calculatedHood;
        }
    }

    // ========================================
    // INTAKE
    // ========================================

    private void handleIntake() {
        if (gamepad1.right_trigger > 0.1) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
        }
    }

    // ========================================
    // DEBUG OUTPUT
    // ========================================

    private void handleDebugOutput() {
        // GP1-A: Print lookup table
        if (gamepad1.a && !lastGP1_A) {
            String table = ContinuousShootingCalculator.generateLookupTable();
            String[] lines = table.split("\n");
            for (String line : lines) {
                telemetry.log().add(line);
            }
        }
        lastGP1_A = gamepad1.a;

        // GP1-B: Print equation comparison
        if (gamepad1.b && !lastGP1_B) {
            String comparison = ContinuousShootingCalculator.compareWithRanges();
            String[] lines = comparison.split("\n");
            for (String line : lines) {
                telemetry.log().add(line);
            }
        }
        lastGP1_B = gamepad1.b;
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void displayTelemetry() {
        telemetry.addLine("=== CONTINUOUS SHOOTING TEST ===");
        telemetry.addLine();

        // Distance and calculations
        telemetry.addLine("--- LIMELIGHT DISTANCE ---");
        telemetry.addData("Distance", "%.2f ft (%.1f in)", currentDistance, currentDistance * 12);
        telemetry.addLine();

        // Equation calculations
        telemetry.addLine("--- EQUATION CALCULATIONS ---");
        telemetry.addData("Calculated RPM", "%.0f", calculatedRPM);
        telemetry.addData("Calculated Hood", "%.3f", calculatedHood);
        telemetry.addData("Mode", useEquations ? "AUTO (Equations)" : "MANUAL (Override)");
        telemetry.addLine();

        // Current shooter status
        telemetry.addLine("--- SHOOTER STATUS ---");
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        if (flywheelOn) {
            telemetry.addData("Target RPM", "%.0f", launcher.getTargetRPM());
            telemetry.addData("Actual RPM", "%.0f", launcher.getCurrentRPM());
            double error = Math.abs(launcher.getTargetRPM() - launcher.getCurrentRPM());
            telemetry.addData("Error", "%.0f RPM", error);
            telemetry.addData("Ready", error < 100 ? "YES ✓" : "Spinning up...");
        }
        telemetry.addData("Hood Position", "%.3f", flywheelOn ? launcher.getTargetRPM() > 0 ?
            (useEquations ? calculatedHood : manualHood) : 0 : 0);
        telemetry.addLine();

        // Turret status
        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Auto-Lock", turretLockEnabled ? "ON" : "OFF");
        if (turretLockEnabled) {
            telemetry.addData("Locked", turret.isLocked() ? "YES ✓" : "Tracking...");
        }
        telemetry.addLine();

        // Manual override status
        if (!useEquations) {
            telemetry.addLine(">>> MANUAL OVERRIDE ACTIVE <<<");
            telemetry.addData("Manual RPM", "%.0f", manualRPM);
            telemetry.addData("Manual Hood", "%.3f", manualHood);
            telemetry.addLine("Press GP2-B to return to equations");
            telemetry.addLine();
        }

        // Equations reference
        telemetry.addLine("--- EQUATIONS ---");
        telemetry.addData("RPM", ContinuousShootingCalculator.getRPMEquation());
        telemetry.addData("Hood", ContinuousShootingCalculator.getHoodEquation());
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("GP1: Y-Flywheel RT-Shoot A-Table B-Compare");
        telemetry.addLine("GP2: A-Lock X-Center B-Reset DPAD-Manual");

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
    }
}

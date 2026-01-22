package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * TeleOp for Testing Shooting Sequence - ALL CONTROLS ON GAMEPAD 1
 *
 * GAMEPAD 1 CONTROLS:
 *
 * SHOOTING MODES:
 * - START: Start automatic shooting sequence
 * - BACK: Stop shooting sequence / Flywheel only mode (toggle)
 *
 * HARDWARE SETTINGS (±0.05):
 * - DPAD UP: Hood position +
 * - DPAD DOWN: Hood position -
 * - DPAD LEFT: Flywheel power -
 * - DPAD RIGHT: Flywheel power +
 * - LEFT STICK X (analog): Turret position (left/right)
 *
 * TIMING ADJUSTMENTS (±0.05s):
 * - LEFT BUMPER: Spinup time +
 * - LEFT TRIGGER: Spinup time -
 * - A: Ball 1 ramp up +
 * - B: Ball 1 ramp up -
 * - X: Ball 1 wait (B1→B2) +
 * - Y: Ball 1 wait (B1→B2) -
 * - RIGHT BUMPER: Ball 2 ramp up +
 * - RIGHT TRIGGER: Ball 2 ramp up -
 * - LEFT STICK Y (hold up): Ball 2 wait (B2→B3) +
 * - LEFT STICK Y (hold down): Ball 2 wait (B2→B3) -
 * - RIGHT STICK Y (hold up): Ball 3 ramp up +
 * - RIGHT STICK Y (hold down): Ball 3 ramp up -
 */
@TeleOp(name = "Shooting Sequence Test", group = "Test")
public class SHOOTINGSEQUENCETEST extends OpMode {

    // ==================== TUNABLE SHOOTING PARAMETERS ====================

    // Hardware Settings
    private double flywheelPower = 1.0;
    private double hoodPosition = 0.65;
    private double turretPosition = 0.55;

    // Timing Parameters (all in seconds)
    private double spinupTime = 2.0;
    private double ball1RampUpTime = 0.15;
    private double ball1WaitAfter = 0.5;
    private double ball2RampUpTime = 0.30;
    private double ball2WaitAfter = 0.5;
    private double ball3RampUpTime = 0.30;

    // Adjustment increments
    private static final double TIME_INCREMENT = 0.05;
    private static final double POSITION_INCREMENT = 0.05;

    // ==================== ROBOT COMPONENTS ====================
    private IntakeTransfer intakeTransfer;
    private Launcher launcher;
    private Servo turretServo;

    // State tracking
    private ElapsedTime shootTimer;
    private ElapsedTime adjustmentTimer;  // For continuous adjustments
    private boolean shootingSequenceActive = false;
    private boolean flywheelOnlyMode = false;  // Just spin flywheel, no shooting

    // Button tracking for START/BACK
    private boolean lastStart = false;
    private boolean lastBack = false;

    // Shooting phase tracking
    private enum ShootPhase {
        SPINUP,
        BALL1_FEEDING,
        BALL1_WAITING,
        BALL2_FEEDING,
        BALL2_WAITING,
        BALL3_FEEDING,
        COMPLETE
    }
    private ShootPhase currentPhase = ShootPhase.SPINUP;

    @Override
    public void init() {
        // Initialize components
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize turret servo
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            turretServo.setPosition(turretPosition);
        } catch (Exception e) {
            turretServo = null;
        }

        shootTimer = new ElapsedTime();
        adjustmentTimer = new ElapsedTime();

        telemetry.addLine("Shooting Sequence Test - Ready!");
        telemetry.addLine("All controls on Gamepad 1");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update launcher
        launcher.update();

        // Set turret position
        if (turretServo != null) {
            turretServo.setPosition(turretPosition);
        }

        // Handle all controls
        handleModeControls();
        handleHardwareAdjustments();
        handleTimingAdjustments();

        // Execute shooting sequence or flywheel-only mode
        if (shootingSequenceActive) {
            performShootingTest();
        } else if (flywheelOnlyMode) {
            performFlywheelOnly();
        }

        // Update detailed telemetry
        updateDetailedTelemetry();
    }

    /**
     * Handle START and BACK buttons for mode control
     */
    private void handleModeControls() {
        boolean currentStart = gamepad1.start;
        boolean currentBack = gamepad1.back;

        // START: Begin shooting sequence
        if (currentStart && !lastStart) {
            if (!shootingSequenceActive) {
                startShootingSequence();
            }
        }

        // BACK: Toggle flywheel-only mode or stop sequence
        if (currentBack && !lastBack) {
            if (shootingSequenceActive) {
                stopShootingSequence();
            } else {
                flywheelOnlyMode = !flywheelOnlyMode;
                if (flywheelOnlyMode) {
                    // Start flywheel without shooting
                    launcher.setPower(flywheelPower);
                    launcher.setHoodPosition(hoodPosition);
                    launcher.setSpinning(true);
                } else {
                    // Turn off flywheel
                    launcher.setSpinning(false);
                    if (launcher.flyWheelMotor != null) {
                        launcher.flyWheelMotor.setPower(0);
                    }
                    if (launcher.flyWheelMotor2 != null) {
                        launcher.flyWheelMotor2.setPower(0);
                    }
                }
            }
        }

        lastStart = currentStart;
        lastBack = currentBack;
    }

    /**
     * Handle hardware adjustments (flywheel, hood, turret)
     */
    private void handleHardwareAdjustments() {
        // DPAD for flywheel power and hood position
        if (gamepad1.dpad_right) {
            flywheelPower = Math.min(1.0, flywheelPower + POSITION_INCREMENT);
        }
        if (gamepad1.dpad_left) {
            flywheelPower = Math.max(0.0, flywheelPower - POSITION_INCREMENT);
        }
        if (gamepad1.dpad_up) {
            hoodPosition = Math.min(1.0, hoodPosition + POSITION_INCREMENT);
            launcher.setHoodPosition(hoodPosition);
        }
        if (gamepad1.dpad_down) {
            hoodPosition = Math.max(0.0, hoodPosition - POSITION_INCREMENT);
            launcher.setHoodPosition(hoodPosition);
        }

        // Left stick X for turret (analog control)
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            turretPosition += gamepad1.left_stick_x * POSITION_INCREMENT * 0.5;
            turretPosition = Math.max(0.0, Math.min(1.0, turretPosition));
        }
    }

    /**
     * Handle timing adjustments
     */
    private void handleTimingAdjustments() {
        // Use timer to prevent too-rapid adjustments
        if (adjustmentTimer.milliseconds() < 150) {
            return;  // Limit adjustment rate to ~7 per second
        }

        boolean adjusted = false;

        // SPINUP TIME (Bumper/Trigger left)
        if (gamepad1.left_bumper) {
            spinupTime += TIME_INCREMENT;
            adjusted = true;
        }
        if (gamepad1.left_trigger > 0.5) {
            spinupTime = Math.max(0, spinupTime - TIME_INCREMENT);
            adjusted = true;
        }

        // BALL 1 RAMP UP (A/B)
        if (gamepad1.a) {
            ball1RampUpTime += TIME_INCREMENT;
            adjusted = true;
        }
        if (gamepad1.b) {
            ball1RampUpTime = Math.max(0, ball1RampUpTime - TIME_INCREMENT);
            adjusted = true;
        }

        // BALL 1 WAIT (X/Y)
        if (gamepad1.x) {
            ball1WaitAfter += TIME_INCREMENT;
            adjusted = true;
        }
        if (gamepad1.y) {
            ball1WaitAfter = Math.max(0, ball1WaitAfter - TIME_INCREMENT);
            adjusted = true;
        }

        // BALL 2 RAMP UP (Bumper/Trigger right)
        if (gamepad1.right_bumper) {
            ball2RampUpTime += TIME_INCREMENT;
            adjusted = true;
        }
        if (gamepad1.right_trigger > 0.5) {
            ball2RampUpTime = Math.max(0, ball2RampUpTime - TIME_INCREMENT);
            adjusted = true;
        }

        // BALL 2 WAIT (Left stick Y)
        if (gamepad1.left_stick_y > 0.5) {
            ball2WaitAfter += TIME_INCREMENT;
            adjusted = true;
        }
        if (gamepad1.left_stick_y < -0.5) {
            ball2WaitAfter = Math.max(0, ball2WaitAfter - TIME_INCREMENT);
            adjusted = true;
        }

        // BALL 3 RAMP UP (Right stick Y)
        if (gamepad1.right_stick_y > 0.5) {
            ball3RampUpTime += TIME_INCREMENT;
            adjusted = true;
        }
        if (gamepad1.right_stick_y < -0.5) {
            ball3RampUpTime = Math.max(0, ball3RampUpTime - TIME_INCREMENT);
            adjusted = true;
        }

        if (adjusted) {
            adjustmentTimer.reset();
        }
    }

    /**
     * Start the shooting sequence
     */
    private void startShootingSequence() {
        shootingSequenceActive = true;
        flywheelOnlyMode = false;
        shootTimer.reset();
        currentPhase = ShootPhase.SPINUP;

        // Start flywheel
        launcher.setPower(flywheelPower);
        launcher.setHoodPosition(hoodPosition);
        launcher.setSpinning(true);
    }

    /**
     * Stop the shooting sequence
     */
    private void stopShootingSequence() {
        shootingSequenceActive = false;

        // Turn everything off
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
     * Flywheel-only mode - just spin flywheel, no shooting
     */
    private void performFlywheelOnly() {
        // Keep flywheel spinning at current settings
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(flywheelPower);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(flywheelPower);
        }
        launcher.setHoodPosition(hoodPosition);
        launcher.setSpinning(true);

        // Keep ramp down and intake off
        intakeTransfer.transferDown();
        intakeTransfer.stopIntake();
    }

    /**
     * Shooting sequence with adjustable timing
     */
    private void performShootingTest() {
        // Keep flywheel spinning
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(flywheelPower);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(flywheelPower);
        }
        launcher.setHoodPosition(hoodPosition);
        launcher.setSpinning(true);

        double elapsed = shootTimer.seconds();

        // Calculate cumulative times
        double ball1Start = spinupTime;
        double ball1End = ball1Start + ball1RampUpTime;
        double ball2Start = ball1End + ball1WaitAfter;
        double ball2End = ball2Start + ball2RampUpTime;
        double ball3Start = ball2End + ball2WaitAfter;
        double ball3End = ball3Start + ball3RampUpTime;

        // State machine
        if (elapsed < spinupTime) {
            currentPhase = ShootPhase.SPINUP;
            intakeTransfer.transferDown();
            intakeTransfer.stopIntake();
        }
        else if (elapsed < ball1End) {
            currentPhase = ShootPhase.BALL1_FEEDING;
            intakeTransfer.transferUp();
            intakeTransfer.startIntake();
        }
        else if (elapsed < ball2Start) {
            currentPhase = ShootPhase.BALL1_WAITING;
            intakeTransfer.transferDown();
            intakeTransfer.startIntake();
        }
        else if (elapsed < ball2End) {
            currentPhase = ShootPhase.BALL2_FEEDING;
            intakeTransfer.transferUp();
            intakeTransfer.startIntake();
        }
        else if (elapsed < ball3Start) {
            currentPhase = ShootPhase.BALL2_WAITING;
            intakeTransfer.transferDown();
            intakeTransfer.startIntake();
        }
        else if (elapsed < ball3End) {
            currentPhase = ShootPhase.BALL3_FEEDING;
            intakeTransfer.transferUp();
            intakeTransfer.startIntake();
        }
        else {
            currentPhase = ShootPhase.COMPLETE;
            intakeTransfer.transferDown();
            intakeTransfer.stopIntake();

            if (elapsed > ball3End + 1.0) {
                stopShootingSequence();
            }
        }
    }

    /**
     * DETAILED telemetry display with all controls visible
     */
    private void updateDetailedTelemetry() {
        telemetry.clear();

        // Header
        telemetry.addLine("╔════════════════════════════════════════╗");
        telemetry.addLine("║   SHOOTING SEQUENCE TEST - GP1 ONLY   ║");
        telemetry.addLine("╚════════════════════════════════════════╝");
        telemetry.addLine();

        // Current Mode
        telemetry.addLine("━━━━━━━━━━ MODE ━━━━━━━━━━");
        if (shootingSequenceActive) {
            telemetry.addLine("🔥 SHOOTING SEQUENCE ACTIVE");
            telemetry.addData("  Phase", currentPhase);
            telemetry.addData("  Elapsed", "%.2fs", shootTimer.seconds());
        } else if (flywheelOnlyMode) {
            telemetry.addLine("🌀 FLYWHEEL ONLY MODE");
        } else {
            telemetry.addLine("⏸  STANDBY");
        }
        telemetry.addLine();

        // Hardware Settings
        telemetry.addLine("━━━━━ HARDWARE SETTINGS ━━━━━");
        telemetry.addData("Flywheel", "%.2f  [DPAD ←→]", flywheelPower);
        telemetry.addData("Hood", "%.2f  [DPAD ↑↓]", hoodPosition);
        telemetry.addData("Turret", "%.2f  [L-STICK X]", turretPosition);
        telemetry.addLine();

        // Timing Settings
        telemetry.addLine("━━━━━━ TIMING (seconds) ━━━━━━");
        telemetry.addData("Spinup", "%.2f  [LB / LT]", spinupTime);
        telemetry.addLine();
        telemetry.addData("Ball 1 Ramp", "%.2f  [A / B]", ball1RampUpTime);
        telemetry.addData("Wait B1→B2", "%.2f  [X / Y]", ball1WaitAfter);
        telemetry.addLine();
        telemetry.addData("Ball 2 Ramp", "%.2f  [RB / RT]", ball2RampUpTime);
        telemetry.addData("Wait B2→B3", "%.2f  [L-STICK Y]", ball2WaitAfter);
        telemetry.addLine();
        telemetry.addData("Ball 3 Ramp", "%.2f  [R-STICK Y]", ball3RampUpTime);
        telemetry.addLine();

        // Total Time
        double totalTime = spinupTime + ball1RampUpTime + ball1WaitAfter +
                ball2RampUpTime + ball2WaitAfter + ball3RampUpTime + 1.0;
        telemetry.addData("⏱ Total Time", "~%.2fs", totalTime);
        telemetry.addLine();

        // Controls Reference
        telemetry.addLine("━━━━━━━━ CONTROLS ━━━━━━━━");
        telemetry.addLine("START = Begin Sequence");
        telemetry.addLine("BACK  = Stop / Flywheel Only");
        telemetry.addLine();

        // Current inputs (helpful for debugging)
        telemetry.addLine("━━━━━ CURRENT INPUTS ━━━━━");
        telemetry.addData("L-Stick", "X:%.2f Y:%.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("R-Stick", "X:%.2f Y:%.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Triggers", "L:%.2f R:%.2f", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Bumpers", "L:%s R:%s",
                gamepad1.left_bumper ? "■" : "□",
                gamepad1.right_bumper ? "■" : "□");
        telemetry.addData("Buttons", "A:%s B:%s X:%s Y:%s",
                gamepad1.a ? "■" : "□",
                gamepad1.b ? "■" : "□",
                gamepad1.x ? "■" : "□",
                gamepad1.y ? "■" : "□");
        telemetry.addData("DPAD", "↑:%s ↓:%s ←:%s →:%s",
                gamepad1.dpad_up ? "■" : "□",
                gamepad1.dpad_down ? "■" : "□",
                gamepad1.dpad_left ? "■" : "□",
                gamepad1.dpad_right ? "■" : "□");

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
    }
}
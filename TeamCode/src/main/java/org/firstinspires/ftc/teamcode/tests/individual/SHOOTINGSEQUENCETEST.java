package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * TeleOp for Testing Shooting Sequence
 *
 * TUNABLE PARAMETERS - LIVE ADJUSTMENT:
 *
 * GAMEPAD 1 (Test Execution & Manual Controls):
 * - A: Start automatic shooting sequence
 * - B: Stop shooting sequence
 * - RIGHT TRIGGER: Intake ON (hold)
 * - LEFT TRIGGER: Eject ON (hold)
 * - DPAD UP: Ramp UP
 * - DPAD DOWN: Ramp DOWN
 * - X: Increase flywheel power (+0.05)
 * - Y: Decrease flywheel power (-0.05)
 * - RIGHT BUMPER: Increase hood position (+0.05)
 * - LEFT BUMPER: Decrease hood position (-0.05)
 *
 * GAMEPAD 2 (Live Timing Adjustments - ±0.05s per press):
 * - DPAD UP/DOWN: Spinup Time
 * - A/B: Ball 1 Ramp Up Time
 * - X/Y: Ball 1 Wait After
 * - LEFT/RIGHT BUMPER: Ball 2 Ramp Up Time
 * - LEFT STICK UP/DOWN: Ball 2 Wait After
 * - RIGHT STICK UP/DOWN: Ball 3 Ramp Up Time
 */
@TeleOp(name = "Shooting Sequence Test", group = "Test")
public class SHOOTINGSEQUENCETEST extends OpMode {

    // ==================== TUNABLE SHOOTING PARAMETERS ====================
    // Adjust these values to tune the shooting sequence

    // Flywheel and Hood Settings (Gamepad 1)
    private double flywheelPower = 1.0;        // X/Y buttons
    private double hoodPosition = 0.65;        // Bumpers
    private static final double TURRET_POSITION = 0.55;      // Locked position

    // ==================== TIMING PARAMETERS (GAMEPAD 2) ====================
    // All times in seconds - adjustable live with gamepad 2

    // Adjustment increment
    private static final double TIME_INCREMENT = 0.05;  // 50ms per button press

    // Spin-up time before first ball (DPAD UP/DOWN)
    private double spinupTime = 2.0;

    // BALL 1 timing (A/B and X/Y)
    private double ball1RampUpTime = 0.15;      // How long ramp stays UP (A/B)
    private double ball1WaitAfter = 0.5;         // Wait after ramp goes DOWN (X/Y)

    // BALL 2 timing (Left/Right Bumpers and Left/Right Stick Y)
    private double ball2RampUpTime = 0.30;      // How long ramp stays UP (Bumpers)
    private double ball2WaitAfter = 0.5;         // Wait after ramp goes DOWN (Left Stick Y)

    // BALL 3 timing (Right Stick Y)
    private double ball3RampUpTime = 0.30;      // How long ramp stays UP (Right Stick Y)

    // ==================== ROBOT COMPONENTS ====================
    private IntakeTransfer intakeTransfer;
    private Launcher launcher;
    private Servo turretServo;

    // State tracking
    private ElapsedTime shootTimer;
    private boolean shootingSequenceActive = false;

    // Gamepad 1 button tracking
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastXButton = false;
    private boolean lastYButton = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    // Gamepad 2 button tracking (for timing adjustments)
    private boolean lastG2A = false;
    private boolean lastG2B = false;
    private boolean lastG2X = false;
    private boolean lastG2Y = false;
    private boolean lastG2DpadUp = false;
    private boolean lastG2DpadDown = false;
    private boolean lastG2LeftBumper = false;
    private boolean lastG2RightBumper = false;
    private boolean lastG2LeftStickUp = false;
    private boolean lastG2LeftStickDown = false;
    private boolean lastG2RightStickUp = false;
    private boolean lastG2RightStickDown = false;

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
            turretServo.setPosition(TURRET_POSITION);
            telemetry.addLine("✓ Turret Servo: OK");
        } catch (Exception e) {
            turretServo = null;
            telemetry.addLine("✗ Turret Servo: NOT FOUND");
        }

        shootTimer = new ElapsedTime();

        telemetry.addLine("=================================");
        telemetry.addLine("   SHOOTING SEQUENCE TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 1 - Test Controls:");
        telemetry.addLine("  A: Start | B: Stop");
        telemetry.addLine("  RT: Intake | LT: Eject");
        telemetry.addLine("  DPAD ↑↓: Ramp");
        telemetry.addLine("  X/Y: Flywheel | Bumpers: Hood");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 2 - Live Timing (±0.05s):");
        telemetry.addLine("  DPAD ↑↓: Spinup Time");
        telemetry.addLine("  A/B: Ball 1 Ramp Up");
        telemetry.addLine("  X/Y: Ball 1 Wait");
        telemetry.addLine("  Bumpers: Ball 2 Ramp Up");
        telemetry.addLine("  L-Stick ↑↓: Ball 2 Wait");
        telemetry.addLine("  R-Stick ↑↓: Ball 3 Ramp Up");
        telemetry.addLine();
        telemetry.addData("Flywheel", "%.2f", flywheelPower);
        telemetry.addData("Hood", "%.2f", hoodPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update launcher
        launcher.update();

        // Keep turret locked
        if (turretServo != null) {
            turretServo.setPosition(TURRET_POSITION);
        }

        // Handle shooting sequence controls (gamepad 1)
        handleShootingControls();

        // Handle manual controls (gamepad 1)
        handleManualControls();

        // Handle timing adjustments (gamepad 2)
        handleTimingAdjustments();

        // Execute shooting sequence if active
        if (shootingSequenceActive) {
            performShootingTest();
        }

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Handle shooting sequence start/stop (Gamepad 1)
     */
    private void handleShootingControls() {
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;
        boolean currentX = gamepad1.x;
        boolean currentY = gamepad1.y;
        boolean currentRightBumper = gamepad1.right_bumper;
        boolean currentLeftBumper = gamepad1.left_bumper;

        // A button: Start shooting sequence
        if (currentA && !lastAButton) {
            startShootingSequence();
        }

        // B button: Stop shooting sequence
        if (currentB && !lastBButton) {
            stopShootingSequence();
        }

        // X button: Increase flywheel power
        if (currentX && !lastXButton) {
            flywheelPower = Math.min(1.0, flywheelPower + 0.05);
        }

        // Y button: Decrease flywheel power
        if (currentY && !lastYButton) {
            flywheelPower = Math.max(0.0, flywheelPower - 0.05);
        }

        // Right bumper: Increase hood position
        if (currentRightBumper && !lastRightBumper) {
            hoodPosition = Math.min(1.0, hoodPosition + 0.05);
            launcher.setHoodPosition(hoodPosition);
        }

        // Left bumper: Decrease hood position
        if (currentLeftBumper && !lastLeftBumper) {
            hoodPosition = Math.max(0.0, hoodPosition - 0.05);
            launcher.setHoodPosition(hoodPosition);
        }

        lastAButton = currentA;
        lastBButton = currentB;
        lastXButton = currentX;
        lastYButton = currentY;
        lastRightBumper = currentRightBumper;
        lastLeftBumper = currentLeftBumper;
    }

    /**
     * Handle manual intake/ramp controls (Gamepad 1)
     */
    private void handleManualControls() {
        // Only allow manual controls when shooting sequence is NOT active
        if (!shootingSequenceActive) {
            // Right trigger: Intake
            if (gamepad1.right_trigger > 0.1) {
                intakeTransfer.startIntake();
            } else if (gamepad1.left_trigger < 0.1) {
                intakeTransfer.stopIntake();
            }

            // Left trigger: Eject
            if (gamepad1.left_trigger > 0.1) {
                intakeTransfer.startEject(1.0);
            }

            // Dpad up: Ramp up
            if (gamepad1.dpad_up) {
                intakeTransfer.transferUp();
            }

            // Dpad down: Ramp down
            if (gamepad1.dpad_down) {
                intakeTransfer.transferDown();
            }
        }
    }

    /**
     * Handle timing adjustments with Gamepad 2
     * All adjustments are ±0.05 seconds (50ms) per button press
     */
    private void handleTimingAdjustments() {
        // Read all gamepad 2 inputs
        boolean g2A = gamepad2.a;
        boolean g2B = gamepad2.b;
        boolean g2X = gamepad2.x;
        boolean g2Y = gamepad2.y;
        boolean g2DpadUp = gamepad2.dpad_up;
        boolean g2DpadDown = gamepad2.dpad_down;
        boolean g2LeftBumper = gamepad2.left_bumper;
        boolean g2RightBumper = gamepad2.right_bumper;
        boolean g2LeftStickUp = gamepad2.left_stick_y > 0.5;
        boolean g2LeftStickDown = gamepad2.left_stick_y < -0.5;
        boolean g2RightStickUp = gamepad2.right_stick_y > 0.5;
        boolean g2RightStickDown = gamepad2.right_stick_y < -0.5;

        // SPINUP TIME (DPAD UP/DOWN)
        if (g2DpadUp && !lastG2DpadUp) {
            spinupTime += TIME_INCREMENT;
        }
        if (g2DpadDown && !lastG2DpadDown) {
            spinupTime = Math.max(0, spinupTime - TIME_INCREMENT);
        }

        // BALL 1 RAMP UP TIME (A/B)
        if (g2A && !lastG2A) {
            ball1RampUpTime += TIME_INCREMENT;
        }
        if (g2B && !lastG2B) {
            ball1RampUpTime = Math.max(0, ball1RampUpTime - TIME_INCREMENT);
        }

        // BALL 1 WAIT AFTER (X/Y)
        if (g2X && !lastG2X) {
            ball1WaitAfter += TIME_INCREMENT;
        }
        if (g2Y && !lastG2Y) {
            ball1WaitAfter = Math.max(0, ball1WaitAfter - TIME_INCREMENT);
        }

        // BALL 2 RAMP UP TIME (BUMPERS)
        if (g2RightBumper && !lastG2RightBumper) {
            ball2RampUpTime += TIME_INCREMENT;
        }
        if (g2LeftBumper && !lastG2LeftBumper) {
            ball2RampUpTime = Math.max(0, ball2RampUpTime - TIME_INCREMENT);
        }

        // BALL 2 WAIT AFTER (LEFT STICK Y)
        if (g2LeftStickUp && !lastG2LeftStickUp) {
            ball2WaitAfter += TIME_INCREMENT;
        }
        if (g2LeftStickDown && !lastG2LeftStickDown) {
            ball2WaitAfter = Math.max(0, ball2WaitAfter - TIME_INCREMENT);
        }

        // BALL 3 RAMP UP TIME (RIGHT STICK Y)
        if (g2RightStickUp && !lastG2RightStickUp) {
            ball3RampUpTime += TIME_INCREMENT;
        }
        if (g2RightStickDown && !lastG2RightStickDown) {
            ball3RampUpTime = Math.max(0, ball3RampUpTime - TIME_INCREMENT);
        }

        // Update last states
        lastG2A = g2A;
        lastG2B = g2B;
        lastG2X = g2X;
        lastG2Y = g2Y;
        lastG2DpadUp = g2DpadUp;
        lastG2DpadDown = g2DpadDown;
        lastG2LeftBumper = g2LeftBumper;
        lastG2RightBumper = g2RightBumper;
        lastG2LeftStickUp = g2LeftStickUp;
        lastG2LeftStickDown = g2LeftStickDown;
        lastG2RightStickUp = g2RightStickUp;
        lastG2RightStickDown = g2RightStickDown;
    }

    /**
     * Start the shooting sequence
     */
    private void startShootingSequence() {
        shootingSequenceActive = true;
        shootTimer.reset();
        currentPhase = ShootPhase.SPINUP;

        // Start flywheel
        launcher.setPower(flywheelPower);
        launcher.setHoodPosition(hoodPosition);
        launcher.setSpinning(true);

        telemetry.addLine("🔥 SHOOTING SEQUENCE STARTED!");
        telemetry.update();
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

        telemetry.addLine("🛑 SHOOTING SEQUENCE STOPPED");
        telemetry.update();
    }

    /**
     * Test version of performShooting() with adjustable timing
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

        // Calculate cumulative times for each phase
        double ball1Start = spinupTime;
        double ball1End = ball1Start + ball1RampUpTime;
        double ball2Start = ball1End + ball1WaitAfter;
        double ball2End = ball2Start + ball2RampUpTime;
        double ball3Start = ball2End + ball2WaitAfter;
        double ball3End = ball3Start + ball3RampUpTime;

        // PHASE: Spin-up
        if (elapsed < spinupTime) {
            currentPhase = ShootPhase.SPINUP;
            intakeTransfer.transferDown();
            intakeTransfer.stopIntake();
        }
        // BALL 1: Ramp UP, Intake ON
        else if (elapsed < ball1End) {
            currentPhase = ShootPhase.BALL1_FEEDING;
            intakeTransfer.transferUp();
            intakeTransfer.startIntake();
        }
        // BALL 1: Ramp DOWN, wait
        else if (elapsed < ball2Start) {
            currentPhase = ShootPhase.BALL1_WAITING;
            intakeTransfer.transferDown();
            intakeTransfer.startIntake();  // Keep intake running
        }
        // BALL 2: Ramp UP, Intake ON
        else if (elapsed < ball2End) {
            currentPhase = ShootPhase.BALL2_FEEDING;
            intakeTransfer.transferUp();
            intakeTransfer.startIntake();
        }
        // BALL 2: Ramp DOWN, wait
        else if (elapsed < ball3Start) {
            currentPhase = ShootPhase.BALL2_WAITING;
            intakeTransfer.transferDown();
            intakeTransfer.startIntake();  // Keep intake running
        }
        // BALL 3: Ramp UP, Intake ON
        else if (elapsed < ball3End) {
            currentPhase = ShootPhase.BALL3_FEEDING;
            intakeTransfer.transferUp();
            intakeTransfer.startIntake();
        }
        // BALL 3: Complete - Ramp DOWN
        else {
            currentPhase = ShootPhase.COMPLETE;
            intakeTransfer.transferDown();
            intakeTransfer.stopIntake();

            // Auto-stop after complete (add 1 second buffer)
            if (elapsed > ball3End + 1.0) {
                stopShootingSequence();
            }
        }
    }

    /**
     * Update telemetry with all relevant information
     */
    private void updateTelemetry() {
        telemetry.addLine("=================================");
        telemetry.addLine("   SHOOTING SEQUENCE TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();

        // Current settings
        telemetry.addLine("--- SHOOTER SETTINGS (GP1) ---");
        telemetry.addData("Flywheel", "%.2f (X/Y)", flywheelPower);
        telemetry.addData("Hood", "%.2f (Bumpers)", hoodPosition);
        telemetry.addData("Turret", "%.2f (locked)", TURRET_POSITION);
        telemetry.addLine();

        // Timing parameters - LIVE ADJUSTABLE
        telemetry.addLine("--- TIMING (GP2: ±0.05s) ---");
        telemetry.addData("Spinup", "%.2fs (DPAD ↑↓)", spinupTime);
        telemetry.addLine();
        telemetry.addData("Ball 1 Ramp", "%.2fs (A/B)", ball1RampUpTime);
        telemetry.addData("Ball 1 Wait", "%.2fs (X/Y)", ball1WaitAfter);
        telemetry.addLine();
        telemetry.addData("Ball 2 Ramp", "%.2fs (Bumpers)", ball2RampUpTime);
        telemetry.addData("Ball 2 Wait", "%.2fs (L-Stick)", ball2WaitAfter);
        telemetry.addLine();
        telemetry.addData("Ball 3 Ramp", "%.2fs (R-Stick)", ball3RampUpTime);
        telemetry.addLine();

        // Calculate total time
        double totalTime = spinupTime + ball1RampUpTime + ball1WaitAfter +
                ball2RampUpTime + ball2WaitAfter + ball3RampUpTime + 1.0;
        telemetry.addData("Total Time", "~%.2fs", totalTime);
        telemetry.addLine();

        // Shooting status
        telemetry.addLine("--- SHOOTING STATUS ---");
        telemetry.addData("Active", shootingSequenceActive ? "YES 🔥" : "NO");
        if (shootingSequenceActive) {
            telemetry.addData("Phase", currentPhase);
            telemetry.addData("Elapsed", "%.2fs", shootTimer.seconds());
            telemetry.addData("Flywheel", launcher.isSpinning() ? "ON" : "OFF");
        }
        telemetry.addLine();

        // Manual controls status
        if (!shootingSequenceActive) {
            telemetry.addLine("--- MANUAL CONTROLS (GP1) ---");
            telemetry.addData("RT (Intake)", gamepad1.right_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("LT (Eject)", gamepad1.left_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("Ramp", gamepad1.dpad_up ? "UP" : (gamepad1.dpad_down ? "DOWN" : "---"));
            telemetry.addLine();
        }

        // Quick reference
        telemetry.addLine("=================================");
        telemetry.addLine("GP1: A=Start | B=Stop");
        telemetry.addLine("GP2: Adjust timing live!");
        telemetry.addLine("=================================");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown
        if (launcher != null) {
            launcher.setSpinning(false);
        }
        if (intakeTransfer != null) {
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
        }
    }
}
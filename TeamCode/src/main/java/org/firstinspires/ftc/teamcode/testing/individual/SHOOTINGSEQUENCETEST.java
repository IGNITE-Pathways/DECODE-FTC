package org.firstinspires.ftc.teamcode.tests.individual;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Launcher;

/**
 * TeleOp Testing Program for Shooting Sequence
 *
 * This program allows real-time tuning of all shooting parameters with gamepad controls.
 *
 * GAMEPAD 1 CONTROLS:
 * ==================
 *
 * SHOOTING CONTROL:
 * - A Button: START shooting sequence
 * - B Button: STOP shooting sequence (emergency stop)
 *
 * MANUAL INTAKE/EJECT (when NOT shooting):
 * - RIGHT TRIGGER: Manual intake (hold to run)
 * - LEFT TRIGGER: Manual eject (hold to run)
 *
 * PARAMETER SELECTION (D-PAD):
 * - D-PAD LEFT: Previous parameter
 * - D-PAD RIGHT: Next parameter
 *
 * VALUE ADJUSTMENT:
 * - LEFT BUMPER: Decrease selected parameter
 * - RIGHT BUMPER: Increase selected parameter
 *
 * PARAMETER LIST (cycle through with D-PAD LEFT/RIGHT):
 * 1. BALL1_FLYWHEEL_POWER (±0.01 increments, 0.0-1.0 range)
 * 2. BALL1_HOOD_POSITION (±0.01 increments, 0.0-1.0 range)
 * 3. BALL2_FLYWHEEL_POWER (±0.01 increments, 0.0-1.0 range)
 * 4. BALL2_HOOD_POSITION (±0.01 increments, 0.0-1.0 range)
 * 5. BALL3_FLYWHEEL_POWER (±0.01 increments, 0.0-1.0 range)
 * 6. BALL3_HOOD_POSITION (±0.01 increments, 0.0-1.0 range)
 * 7. TURRET_LOCKED_POSITION (±0.01 increments, 0.0-1.0 range)
 * 8. BALL1_FEED_TIME (±0.01s increments)
 * 9. BALL1_FEED_INTAKE_ON (toggle ON/OFF)
 * 10. BALL1_FEED_EJECT_ON (toggle ON/OFF)
 * 11. BALL1_RECOVERY_TIME (±0.01s increments)
 * 12. BALL1_RECOVERY_INTAKE_ON (toggle ON/OFF)
 * 13. BALL1_RECOVERY_EJECT_ON (toggle ON/OFF)
 * 14. BALL2_FEED_TIME (±0.01s increments)
 * 15. BALL2_FEED_INTAKE_ON (toggle ON/OFF)
 * 16. BALL2_FEED_EJECT_ON (toggle ON/OFF)
 * 17. BALL2_RECOVERY_TIME (±0.01s increments)
 * 18. BALL2_RECOVERY_INTAKE_ON (toggle ON/OFF)
 * 19. BALL2_RECOVERY_EJECT_ON (toggle ON/OFF)
 * 20. BALL3_FEED_TIME (±0.01s increments)
 * 21. BALL3_FEED_INTAKE_ON (toggle ON/OFF)
 * 22. BALL3_FEED_EJECT_ON (toggle ON/OFF)
 * 23. FINISH_INTAKE_ON (toggle ON/OFF)
 * 24. FINISH_EJECT_ON (toggle ON/OFF)
 */
@TeleOp(name = "Shooting Sequence Tester", group = "Testing")
public class SHOOTINGSEQUENCETEST extends OpMode {

    // ==================== TUNABLE SHOOTING PARAMETERS ====================
    // These are NOT final so they can be adjusted during runtime

    // Hardware positions - PER BALL
    private double BALL1_FLYWHEEL_POWER = 1.0;
    private double BALL1_HOOD_POSITION = 0.6;

    private double BALL2_FLYWHEEL_POWER = 1.0;
    private double BALL2_HOOD_POSITION = 0.6;

    private double BALL3_FLYWHEEL_POWER = 1.0;
    private double BALL3_HOOD_POSITION = 0.6;

    // Turret position (same for all balls)
    private double TURRET_LOCKED_POSITION = 0.6;

    // Ball 1 timing and control
    private double BALL1_FEED_TIME = 0.15;
    private boolean BALL1_FEED_INTAKE_ON = true;
    private boolean BALL1_FEED_EJECT_ON = false;
    private double BALL1_RECOVERY_TIME = 1.5;
    private boolean BALL1_RECOVERY_INTAKE_ON = true;
    private boolean BALL1_RECOVERY_EJECT_ON = false;

    // Ball 2 timing and control
    private double BALL2_FEED_TIME = 0.15;
    private boolean BALL2_FEED_INTAKE_ON = true;
    private boolean BALL2_FEED_EJECT_ON = false;
    private double BALL2_RECOVERY_TIME = 1.5;
    private boolean BALL2_RECOVERY_INTAKE_ON = true;
    private boolean BALL2_RECOVERY_EJECT_ON = false;

    // Ball 3 timing and control
    private double BALL3_FEED_TIME = 0.15;
    private boolean BALL3_FEED_INTAKE_ON = true;
    private boolean BALL3_FEED_EJECT_ON = false;

    // Finish phase control
    private boolean FINISH_INTAKE_ON = false;
    private boolean FINISH_EJECT_ON = false;

    // Spinup phase (fixed - not adjustable)
    private static final double SPINUP_TIME = 2.0;
    private static final boolean SPINUP_INTAKE_ON = false;
    private static final boolean SPINUP_EJECT_ON = false;

    // ==================== ROBOT COMPONENTS ====================
    private IntakeTransfer intakeTransfer;
    private Launcher launcher;
    private Servo turretServo;

    // ==================== STATE MANAGEMENT ====================
    private Timer shootTimer;
    private boolean isShooting = false;

    // Parameter selection
    private int selectedParameter = 0;
    private static final int NUM_PARAMETERS = 24;

    // Button debouncing
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;

    /**
     * Enum for parameter selection
     */
    private enum Parameter {
        BALL1_FLYWHEEL_POWER(0, "Ball 1 Flywheel Power", "LB/RB: ±0.01"),
        BALL1_HOOD_POSITION(1, "Ball 1 Hood Position", "LB/RB: ±0.01"),
        BALL2_FLYWHEEL_POWER(2, "Ball 2 Flywheel Power", "LB/RB: ±0.01"),
        BALL2_HOOD_POSITION(3, "Ball 2 Hood Position", "LB/RB: ±0.01"),
        BALL3_FLYWHEEL_POWER(4, "Ball 3 Flywheel Power", "LB/RB: ±0.01"),
        BALL3_HOOD_POSITION(5, "Ball 3 Hood Position", "LB/RB: ±0.01"),
        TURRET_POSITION(6, "Turret Position", "LB/RB: ±0.01"),
        BALL1_FEED_TIME(7, "Ball 1 Feed Time", "LB/RB: ±0.01s"),
        BALL1_FEED_INTAKE(8, "Ball 1 Feed Intake", "LB/RB: Toggle"),
        BALL1_FEED_EJECT(9, "Ball 1 Feed Eject", "LB/RB: Toggle"),
        BALL1_RECOVERY_TIME(10, "Ball 1 Recovery Time", "LB/RB: ±0.01s"),
        BALL1_RECOVERY_INTAKE(11, "Ball 1 Recovery Intake", "LB/RB: Toggle"),
        BALL1_RECOVERY_EJECT(12, "Ball 1 Recovery Eject", "LB/RB: Toggle"),
        BALL2_FEED_TIME(13, "Ball 2 Feed Time", "LB/RB: ±0.01s"),
        BALL2_FEED_INTAKE(14, "Ball 2 Feed Intake", "LB/RB: Toggle"),
        BALL2_FEED_EJECT(15, "Ball 2 Feed Eject", "LB/RB: Toggle"),
        BALL2_RECOVERY_TIME(16, "Ball 2 Recovery Time", "LB/RB: ±0.01s"),
        BALL2_RECOVERY_INTAKE(17, "Ball 2 Recovery Intake", "LB/RB: Toggle"),
        BALL2_RECOVERY_EJECT(18, "Ball 2 Recovery Eject", "LB/RB: Toggle"),
        BALL3_FEED_TIME(19, "Ball 3 Feed Time", "LB/RB: ±0.01s"),
        BALL3_FEED_INTAKE(20, "Ball 3 Feed Intake", "LB/RB: Toggle"),
        BALL3_FEED_EJECT(21, "Ball 3 Feed Eject", "LB/RB: Toggle"),
        FINISH_INTAKE(22, "Finish Intake", "LB/RB: Toggle"),
        FINISH_EJECT(23, "Finish Eject", "LB/RB: Toggle");

        private final int index;
        private final String name;
        private final String controls;

        Parameter(int index, String name, String controls) {
            this.index = index;
            this.name = name;
            this.controls = controls;
        }

        public static Parameter fromIndex(int index) {
            for (Parameter p : values()) {
                if (p.index == index) return p;
            }
            return BALL1_FLYWHEEL_POWER;
        }
    }

    @Override
    public void init() {
        // Initialize timers
        shootTimer = new Timer();

        // Initialize subsystems
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // Initialize turret servo
        try {
            turretServo = hardwareMap.get(Servo.class, "turretServo");
            turretServo.setPosition(TURRET_LOCKED_POSITION);
        } catch (Exception e) {
            turretServo = null;
        }

        telemetry.addLine("Shooting Sequence Tester Initialized");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Start Shooting");
        telemetry.addLine("B: Stop Shooting");
        telemetry.addLine("D-PAD L/R: Select Parameter");
        telemetry.addLine("LB/RB: Adjust Value");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update launcher
        launcher.update();

        // Handle parameter selection (D-PAD)
        handleParameterSelection();

        // Handle value adjustment (Bumpers)
        handleValueAdjustment();

        // Handle shooting control (A/B buttons)
        handleShootingControl();

        // Handle manual intake/eject (Triggers) - only when NOT shooting
        if (!isShooting) {
            handleManualIntakeEject();
        }

        // Execute shooting sequence if active
        if (isShooting) {
            performShooting();
        }

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Handle parameter selection with D-PAD
     */
    private void handleParameterSelection() {
        boolean currentDpadLeft = gamepad1.dpad_left;
        boolean currentDpadRight = gamepad1.dpad_right;

        // Previous parameter
        if (currentDpadLeft && !lastDpadLeft) {
            selectedParameter--;
            if (selectedParameter < 0) {
                selectedParameter = NUM_PARAMETERS - 1;
            }
        }

        // Next parameter
        if (currentDpadRight && !lastDpadRight) {
            selectedParameter++;
            if (selectedParameter >= NUM_PARAMETERS) {
                selectedParameter = 0;
            }
        }

        lastDpadLeft = currentDpadLeft;
        lastDpadRight = currentDpadRight;
    }

    /**
     * Handle value adjustment with bumpers
     */
    private void handleValueAdjustment() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentRightBumper = gamepad1.right_bumper;

        Parameter param = Parameter.fromIndex(selectedParameter);

        // Decrease value
        if (currentLeftBumper && !lastLeftBumper) {
            adjustParameter(param, false);
        }

        // Increase value
        if (currentRightBumper && !lastRightBumper) {
            adjustParameter(param, true);
        }

        lastLeftBumper = currentLeftBumper;
        lastRightBumper = currentRightBumper;
    }

    /**
     * Adjust the selected parameter
     */
    private void adjustParameter(Parameter param, boolean increase) {
        double delta = increase ? 0.01 : -0.01;

        switch (param) {
            case BALL1_FLYWHEEL_POWER:
                BALL1_FLYWHEEL_POWER = clamp(BALL1_FLYWHEEL_POWER + delta, 0.0, 1.0);
                break;
            case BALL1_HOOD_POSITION:
                BALL1_HOOD_POSITION = clamp(BALL1_HOOD_POSITION + delta, 0.0, 1.0);
                break;
            case BALL2_FLYWHEEL_POWER:
                BALL2_FLYWHEEL_POWER = clamp(BALL2_FLYWHEEL_POWER + delta, 0.0, 1.0);
                break;
            case BALL2_HOOD_POSITION:
                BALL2_HOOD_POSITION = clamp(BALL2_HOOD_POSITION + delta, 0.0, 1.0);
                break;
            case BALL3_FLYWHEEL_POWER:
                BALL3_FLYWHEEL_POWER = clamp(BALL3_FLYWHEEL_POWER + delta, 0.0, 1.0);
                break;
            case BALL3_HOOD_POSITION:
                BALL3_HOOD_POSITION = clamp(BALL3_HOOD_POSITION + delta, 0.0, 1.0);
                break;
            case TURRET_POSITION:
                TURRET_LOCKED_POSITION = clamp(TURRET_LOCKED_POSITION + delta, 0.0, 1.0);
                if (turretServo != null) {
                    turretServo.setPosition(TURRET_LOCKED_POSITION);
                }
                break;
            case BALL1_FEED_TIME:
                BALL1_FEED_TIME = Math.max(0.0, BALL1_FEED_TIME + delta);
                break;
            case BALL1_FEED_INTAKE:
                BALL1_FEED_INTAKE_ON = !BALL1_FEED_INTAKE_ON;
                break;
            case BALL1_FEED_EJECT:
                BALL1_FEED_EJECT_ON = !BALL1_FEED_EJECT_ON;
                break;
            case BALL1_RECOVERY_TIME:
                BALL1_RECOVERY_TIME = Math.max(0.0, BALL1_RECOVERY_TIME + delta);
                break;
            case BALL1_RECOVERY_INTAKE:
                BALL1_RECOVERY_INTAKE_ON = !BALL1_RECOVERY_INTAKE_ON;
                break;
            case BALL1_RECOVERY_EJECT:
                BALL1_RECOVERY_EJECT_ON = !BALL1_RECOVERY_EJECT_ON;
                break;
            case BALL2_FEED_TIME:
                BALL2_FEED_TIME = Math.max(0.0, BALL2_FEED_TIME + delta);
                break;
            case BALL2_FEED_INTAKE:
                BALL2_FEED_INTAKE_ON = !BALL2_FEED_INTAKE_ON;
                break;
            case BALL2_FEED_EJECT:
                BALL2_FEED_EJECT_ON = !BALL2_FEED_EJECT_ON;
                break;
            case BALL2_RECOVERY_TIME:
                BALL2_RECOVERY_TIME = Math.max(0.0, BALL2_RECOVERY_TIME + delta);
                break;
            case BALL2_RECOVERY_INTAKE:
                BALL2_RECOVERY_INTAKE_ON = !BALL2_RECOVERY_INTAKE_ON;
                break;
            case BALL2_RECOVERY_EJECT:
                BALL2_RECOVERY_EJECT_ON = !BALL2_RECOVERY_EJECT_ON;
                break;
            case BALL3_FEED_TIME:
                BALL3_FEED_TIME = Math.max(0.0, BALL3_FEED_TIME + delta);
                break;
            case BALL3_FEED_INTAKE:
                BALL3_FEED_INTAKE_ON = !BALL3_FEED_INTAKE_ON;
                break;
            case BALL3_FEED_EJECT:
                BALL3_FEED_EJECT_ON = !BALL3_FEED_EJECT_ON;
                break;
            case FINISH_INTAKE:
                FINISH_INTAKE_ON = !FINISH_INTAKE_ON;
                break;
            case FINISH_EJECT:
                FINISH_EJECT_ON = !FINISH_EJECT_ON;
                break;
        }
    }

    /**
     * Handle shooting control with A/B buttons
     */
    private void handleShootingControl() {
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;

        // Start shooting
        if (currentA && !lastAButton && !isShooting) {
            startShooting();
        }

        // Stop shooting
        if (currentB && !lastBButton && isShooting) {
            stopShooting();
        }

        lastAButton = currentA;
        lastBButton = currentB;
    }

    /**
     * Handle manual intake/eject with triggers (only when NOT shooting)
     */
    private void handleManualIntakeEject() {
        float rightTrigger = gamepad1.right_trigger;
        float leftTrigger = gamepad1.left_trigger;

        // Right trigger = intake
        if (rightTrigger > 0.1) {
            intakeTransfer.startIntake();
        }
        // Left trigger = eject
        else if (leftTrigger > 0.1) {
            intakeTransfer.startEject(1.0);
        }
        // No triggers pressed = stop
        else {
            intakeTransfer.stopIntake();
        }
    }

    /**
     * Start shooting sequence
     */
    private void startShooting() {
        // Configure shooter for Ball 1 initially
        launcher.setPower(BALL1_FLYWHEEL_POWER);
        launcher.setHoodPosition(BALL1_HOOD_POSITION);
        launcher.setSpinning(true);

        // Raise transfer ramp and start intake
        intakeTransfer.transferUp();
        intakeTransfer.startIntake();

        // Reset timer and start shooting
        shootTimer.resetTimer();
        isShooting = true;
    }

    /**
     * Stop shooting sequence (emergency stop)
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
        isShooting = false;
    }

    /**
     * Perform shooting sequence (copied from autonomous code)
     * Now with per-ball flywheel power and hood position adjustments
     */
    private void performShooting() {
        // Calculate time markers
        double BALL1_START = SPINUP_TIME;
        double BALL1_END = BALL1_START + BALL1_FEED_TIME;
        double BALL2_START = BALL1_END + BALL1_RECOVERY_TIME;
        double BALL2_END = BALL2_START + BALL2_FEED_TIME;
        double BALL3_START = BALL2_END + BALL2_RECOVERY_TIME;
        double BALL3_END = BALL3_START + BALL3_FEED_TIME;

        // Get current time in shooting sequence
        double elapsed = shootTimer.getElapsedTimeSeconds();

        // Determine which ball we're currently shooting and set appropriate power/hood
        double currentFlywheelPower;
        double currentHoodPosition;

        if (elapsed < BALL2_START) {
            // Ball 1 phase (spinup, feed, recovery)
            currentFlywheelPower = BALL1_FLYWHEEL_POWER;
            currentHoodPosition = BALL1_HOOD_POSITION;
        } else if (elapsed < BALL3_START) {
            // Ball 2 phase (feed, recovery)
            currentFlywheelPower = BALL2_FLYWHEEL_POWER;
            currentHoodPosition = BALL2_HOOD_POSITION;
        } else {
            // Ball 3 phase (feed, finish)
            currentFlywheelPower = BALL3_FLYWHEEL_POWER;
            currentHoodPosition = BALL3_HOOD_POSITION;
        }

        // Apply current flywheel power and hood position
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(currentFlywheelPower);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(currentFlywheelPower);
        }
        launcher.setHoodPosition(currentHoodPosition);
        launcher.setSpinning(true);

        // PHASE 1: SPINUP
        if (elapsed < SPINUP_TIME) {
            intakeTransfer.transferDown();
            if (SPINUP_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (SPINUP_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }

        // PHASE 2: BALL 1 FEED
        else if (elapsed >= BALL1_START && elapsed < BALL1_END) {
            intakeTransfer.transferUp();
            if (BALL1_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL1_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }

        // PHASE 3: BALL 1 RECOVERY
        else if (elapsed >= BALL1_END && elapsed < BALL2_START) {
            intakeTransfer.transferDown();
            if (BALL1_RECOVERY_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL1_RECOVERY_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }

        // PHASE 4: BALL 2 FEED
        else if (elapsed >= BALL2_START && elapsed < BALL2_END) {
            intakeTransfer.transferUp();
            if (BALL2_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL2_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }

        // PHASE 5: BALL 2 RECOVERY
        else if (elapsed >= BALL2_END && elapsed < BALL3_START) {
            intakeTransfer.transferDown();
            if (BALL2_RECOVERY_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL2_RECOVERY_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }

        // PHASE 6: BALL 3 FEED
        else if (elapsed >= BALL3_START && elapsed < BALL3_END) {
            intakeTransfer.transferUp();
            if (BALL3_FEED_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (BALL3_FEED_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }
        }

        // PHASE 7: FINISH
        else if (elapsed >= BALL3_END) {
            intakeTransfer.transferDown();
            if (FINISH_EJECT_ON) {
                intakeTransfer.startEject(1.0);
            } else if (FINISH_INTAKE_ON) {
                intakeTransfer.startIntake();
            } else {
                intakeTransfer.stopIntake();
            }

            // Auto-stop after sequence completes
            stopShooting();
        }
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.addLine("=== SHOOTING TESTER ===");
        telemetry.addData("Status", isShooting ? "SHOOTING" : "IDLE");

        // Show manual intake/eject status when idle
        if (!isShooting) {
            if (gamepad1.right_trigger > 0.1) {
                telemetry.addData("Manual Control", "INTAKE");
            } else if (gamepad1.left_trigger > 0.1) {
                telemetry.addData("Manual Control", "EJECT");
            } else {
                telemetry.addData("Manual Control", "NONE");
            }
        }

        if (isShooting) {
            double elapsed = shootTimer.getElapsedTimeSeconds();
            telemetry.addData("Timer", String.format("%.2fs", elapsed));

            // Show current phase
            double BALL1_START = SPINUP_TIME;
            double BALL1_END = BALL1_START + BALL1_FEED_TIME;
            double BALL2_START = BALL1_END + BALL1_RECOVERY_TIME;
            double BALL2_END = BALL2_START + BALL2_FEED_TIME;
            double BALL3_START = BALL2_END + BALL2_RECOVERY_TIME;
            double BALL3_END = BALL3_START + BALL3_FEED_TIME;

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

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Start | B: Stop");
        if (!isShooting) {
            telemetry.addLine("RT: Intake | LT: Eject");
        }
        telemetry.addLine("D-PAD L/R: Select Param");
        telemetry.addLine("LB/RB: Adjust Value");

        telemetry.addLine();
        telemetry.addLine("=== PARAMETERS ===");

        // Display all parameters with highlighting for selected one
        displayParameter(0, "Ball 1 Flywheel Power", String.format("%.2f", BALL1_FLYWHEEL_POWER));
        displayParameter(1, "Ball 1 Hood Position", String.format("%.2f", BALL1_HOOD_POSITION));
        displayParameter(2, "Ball 2 Flywheel Power", String.format("%.2f", BALL2_FLYWHEEL_POWER));
        displayParameter(3, "Ball 2 Hood Position", String.format("%.2f", BALL2_HOOD_POSITION));
        displayParameter(4, "Ball 3 Flywheel Power", String.format("%.2f", BALL3_FLYWHEEL_POWER));
        displayParameter(5, "Ball 3 Hood Position", String.format("%.2f", BALL3_HOOD_POSITION));
        displayParameter(6, "Turret Position", String.format("%.2f", TURRET_LOCKED_POSITION));
        displayParameter(7, "Ball 1 Feed Time", String.format("%.2fs", BALL1_FEED_TIME));
        displayParameter(8, "Ball 1 Feed Intake", BALL1_FEED_INTAKE_ON ? "ON" : "OFF");
        displayParameter(9, "Ball 1 Feed Eject", BALL1_FEED_EJECT_ON ? "ON" : "OFF");
        displayParameter(10, "Ball 1 Recovery Time", String.format("%.2fs", BALL1_RECOVERY_TIME));
        displayParameter(11, "Ball 1 Recovery Intake", BALL1_RECOVERY_INTAKE_ON ? "ON" : "OFF");
        displayParameter(12, "Ball 1 Recovery Eject", BALL1_RECOVERY_EJECT_ON ? "ON" : "OFF");
        displayParameter(13, "Ball 2 Feed Time", String.format("%.2fs", BALL2_FEED_TIME));
        displayParameter(14, "Ball 2 Feed Intake", BALL2_FEED_INTAKE_ON ? "ON" : "OFF");
        displayParameter(15, "Ball 2 Feed Eject", BALL2_FEED_EJECT_ON ? "ON" : "OFF");
        displayParameter(16, "Ball 2 Recovery Time", String.format("%.2fs", BALL2_RECOVERY_TIME));
        displayParameter(17, "Ball 2 Recovery Intake", BALL2_RECOVERY_INTAKE_ON ? "ON" : "OFF");
        displayParameter(18, "Ball 2 Recovery Eject", BALL2_RECOVERY_EJECT_ON ? "ON" : "OFF");
        displayParameter(19, "Ball 3 Feed Time", String.format("%.2fs", BALL3_FEED_TIME));
        displayParameter(20, "Ball 3 Feed Intake", BALL3_FEED_INTAKE_ON ? "ON" : "OFF");
        displayParameter(21, "Ball 3 Feed Eject", BALL3_FEED_EJECT_ON ? "ON" : "OFF");
        displayParameter(22, "Finish Intake", FINISH_INTAKE_ON ? "ON" : "OFF");
        displayParameter(23, "Finish Eject", FINISH_EJECT_ON ? "ON" : "OFF");

        telemetry.update();
    }

    /**
     * Display a parameter with highlighting if selected
     */
    private void displayParameter(int index, String name, String value) {
        String prefix = (index == selectedParameter) ? ">>> " : "    ";
        telemetry.addData(prefix + name, value);
    }

    /**
     * Clamp a value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        if (isShooting) {
            stopShooting();
        }
    }
}
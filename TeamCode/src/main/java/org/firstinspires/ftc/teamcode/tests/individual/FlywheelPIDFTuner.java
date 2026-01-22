package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Flywheel PIDF Tuning Tool - OPTIMIZED FOR 6000 RPM MOTORS
 *
 * Implements closed-loop velocity control for consistent flywheel performance.
 * This solves the problem of open-loop control not recovering fast enough after shots.
 *
 * RECOMMENDED RPM TARGETS (for 6000 RPM motors):
 * - Close shots (2-4 ft): 2500-3000 RPM
 * - Mid shots (4-6 ft): 3200-3800 RPM
 * - Far shots (6-8 ft): 4000-4500 RPM
 * - Max distance (8+ ft): 4800-5500 RPM
 *
 * NOTE: Start with 3500 RPM and adjust based on your testing
 *
 * CONTROLS:
 * =========
 * Y: Toggle flywheel ON/OFF
 * DPAD UP/DOWN: Adjust target RPM (±50)
 * LEFT STICK Y: Adjust target RPM (fine control)
 *
 * Tuning Gains (hold respective bumper + dpad):
 * LB + DPAD UP/DOWN: Adjust kP (±0.0001)
 * RB + DPAD UP/DOWN: Adjust kF (±0.00001)
 * LB + RB + DPAD UP/DOWN: Adjust kI (±0.00001)
 * A + DPAD UP/DOWN: Adjust kD (±0.0001)
 *
 * RT: Run intake (simulate shot/load)
 * LT: Run eject
 * X: Reset PID
 * B: Save current gains to telemetry
 *
 * TUNING GUIDE:
 * =============
 * 1. Start with kF only - find value where steady-state error is near zero
 * 2. Add kP to handle disturbances (shots) - increase until recovery is fast
 * 3. Add kI to eliminate steady-state error (usually very small)
 * 4. Add kD to reduce overshoot/oscillation (usually very small)
 *
 * CURRENT TUNED VALUES (for 6000 RPM motors):
 * ============================================
 * kF = 0.00020 - Base power (60-70% power at 3500 RPM)
 * kP = 0.0004  - Fast response to RPM drops from shots
 * kI = 0.00003 - Eliminates steady-state error without oscillation
 * kD = 0.0002  - Smooths recovery, reduces overshoot
 *
 * Expected Performance:
 * - Steady-state error: < ±20 RPM
 * - Recovery time after shot: 50-150ms
 * - Overshoot: < 5%
 *
 * VOLTAGE COMPENSATION:
 * ====================
 * NO manual voltage compensation is used. PIDF naturally handles voltage drops:
 * - Battery voltage drops → Flywheel RPM drops → Error increases → PIDF adds more power
 * This is better than artificial voltage compensation which can cause double-compensation.
 * Only emergency shutoff at 10.5V to prevent brownouts.
 *
 * If you have different motors or mechanical setup, use the controls above to fine-tune.
 */
@TeleOp(name = "Tuner: Flywheel PIDF", group = "Tuning")
public class FlywheelPIDFTuner extends LinearOpMode {

    // Hardware
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    private IntakeTransfer intakeTransfer;
    private VoltageSensor voltageSensor;

    // Voltage monitoring (no compensation - PIDF handles it naturally)
    private static final double CRITICAL_VOLTAGE = 10.5; // Emergency shutoff only
    private double currentVoltage = 13.0;

    // PIDF Gains - TUNED FOR 6000 RPM MOTORS
    // These are optimized starting values - fine-tune to your specific robot
    private double kP = 0.0004;   // Proportional - responds to error
    private double kI = 0.00003;  // Integral - eliminates steady-state error
    private double kD = 0.0002;   // Derivative - reduces oscillation
    private double kF = 0.00020;  // Feedforward - provides base power for target velocity

    // Target and control
    private double targetRPM = 3500;  // Target flywheel speed (60% of max for good control)
    private boolean flywheelOn = false;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Velocity measurement (using only motor 1 encoder)
    private int lastPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double currentRPM = 0.0;
    private double avgRPM = 0.0;

    // Velocity smoothing (simple moving average)
    private static final int VELOCITY_SAMPLES = 5;
    private double[] velocitySamples = new double[VELOCITY_SAMPLES];
    private int velocityIndex = 0;

    // Button states
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Telemetry tracking
    private double lastPower = 0.0;
    private double pComponent = 0.0;
    private double iComponent = 0.0;
    private double dComponent = 0.0;
    private double fComponent = 0.0;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("=== FLYWHEEL PIDF TUNER ===");
        telemetry.addLine("Y: Toggle flywheel");
        telemetry.addLine("RT: Intake | LT: Eject");
        telemetry.addLine("DPAD UP/DN: Adjust RPM");
        telemetry.addData("Battery", "%.2fV", currentVoltage);
        telemetry.addLine("PIDF naturally handles voltage drops!");
        telemetry.update();

        waitForStart();

        pidTimer.reset();
        velocityTimer.reset();

        while (opModeIsActive()) {
            handleControls();
            measureVelocity();

            if (flywheelOn) {
                double power = calculatePIDFPower();
                setFlywheelPower(power);
            } else {
                setFlywheelPower(0.0);
                resetPID();
            }

            displayTelemetry();
        }

        setFlywheelPower(0.0);
    }

    // ========================================
    // INITIALIZATION
    // ========================================

    private void initializeHardware() {
        // Initialize flywheel motors as DcMotorEx for velocity control
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR);
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR_2);

        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastPosition = flywheelMotor1.getCurrentPosition();

        // Initialize intake for testing shots
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        // Initialize voltage sensor for brownout prevention
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }
    }

    // ========================================
    // VELOCITY MEASUREMENT
    // ========================================

    private void measureVelocity() {
        double dt = velocityTimer.seconds();
        if (dt < 0.02) return; // Sample at ~50Hz

        int currentPos = flywheelMotor1.getCurrentPosition();

        // Calculate velocity in RPM (28 ticks per revolution for REV motors)
        currentRPM = ((currentPos - lastPosition) / dt) * (60.0 / 28.0);

        // Smooth using moving average
        velocitySamples[velocityIndex] = currentRPM;
        velocityIndex = (velocityIndex + 1) % VELOCITY_SAMPLES;

        double sum = 0;
        for (double sample : velocitySamples) {
            sum += sample;
        }
        avgRPM = sum / VELOCITY_SAMPLES;

        lastPosition = currentPos;
        velocityTimer.reset();
    }

    // ========================================
    // PIDF CONTROLLER
    // ========================================

    private double calculatePIDFPower() {
        // Update voltage reading
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }

        // CRITICAL: Emergency shutoff if voltage too low
        if (currentVoltage < CRITICAL_VOLTAGE) {
            resetPID();
            return 0.0;
        }

        double error = targetRPM - avgRPM;
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Prevent huge jumps on first iteration
        if (dt > 1.0) dt = 0.02;

        // Feedforward - base power proportional to target velocity
        fComponent = kF * targetRPM;

        // Proportional - respond to current error
        pComponent = kP * error;

        // Integral - accumulate error over time (eliminate steady-state error)
        integral += error * dt;
        // Anti-windup: clamp integral
        integral = Math.max(-1000, Math.min(1000, integral));
        iComponent = kI * integral;

        // Derivative - rate of change of error (reduce oscillation)
        double derivative = (error - lastError) / dt;
        dComponent = kD * derivative;

        lastError = error;

        // Combine all components
        double power = fComponent + pComponent + iComponent + dComponent;

        // NO voltage compensation - PIDF naturally compensates for voltage drops
        // by increasing power when RPM drops. Adding voltage comp causes double-compensation.

        // Clamp to valid range
        power = Math.max(0.0, Math.min(1.0, power));

        lastPower = power;
        return power;
    }

    private void resetPID() {
        integral = 0.0;
        lastError = 0.0;
        pComponent = 0.0;
        iComponent = 0.0;
        dComponent = 0.0;
        fComponent = 0.0;
        pidTimer.reset();
    }

    // ========================================
    // CONTROLS
    // ========================================

    private void handleControls() {
        // Toggle flywheel ON/OFF
        if (gamepad1.y && !lastY) {
            flywheelOn = !flywheelOn;
            if (!flywheelOn) {
                resetPID();
            }
        }
        lastY = gamepad1.y;

        // Reset PID
        if (gamepad1.x && !lastX) {
            resetPID();
        }
        lastX = gamepad1.x;

        // Intake control - RT intake, LT eject
        if (gamepad1.right_trigger > 0.1) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
        }

        // Target RPM adjustment
        if (gamepad1.dpad_up && !lastDpadUp) {
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                kI += 0.00001; // Tune kI
            } else if (gamepad1.left_bumper) {
                kP += 0.0001; // Tune kP
            } else if (gamepad1.right_bumper) {
                kF += 0.00001; // Tune kF
            } else if (gamepad1.a) {
                kD += 0.0001; // Tune kD
            } else {
                targetRPM += 50; // Adjust target
            }
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                kI -= 0.00001; // Tune kI
            } else if (gamepad1.left_bumper) {
                kP -= 0.0001; // Tune kP
            } else if (gamepad1.right_bumper) {
                kF -= 0.00001; // Tune kF
            } else if (gamepad1.a) {
                kD -= 0.0001; // Tune kD
            } else {
                targetRPM -= 50; // Adjust target
            }
        }
        lastDpadDown = gamepad1.dpad_down;

        // Fine RPM control with left stick
        targetRPM += -gamepad1.left_stick_y * 5; // ±5 RPM per frame
        targetRPM = Math.max(0, Math.min(6000, targetRPM));

        // Save gains to telemetry
        if (gamepad1.b && !lastB) {
            telemetry.log().add("SAVED GAINS:");
            telemetry.log().add(String.format("kP = %.6f", kP));
            telemetry.log().add(String.format("kI = %.6f", kI));
            telemetry.log().add(String.format("kD = %.6f", kD));
            telemetry.log().add(String.format("kF = %.6f", kF));
            telemetry.log().add(String.format("Target RPM = %.0f", targetRPM));
        }
        lastB = gamepad1.b;
    }

    // ========================================
    // MOTOR CONTROL
    // ========================================

    private void setFlywheelPower(double power) {
        flywheelMotor1.setPower(power);
        flywheelMotor2.setPower(power);
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void displayTelemetry() {
        telemetry.addLine("=== FLYWHEEL PIDF TUNER ===");
        telemetry.addData("Status", flywheelOn ? "RUNNING" : "STOPPED");

        // Battery status - color code with warnings
        String voltageStatus;
        if (currentVoltage < CRITICAL_VOLTAGE) {
            voltageStatus = String.format("!! CRITICAL: %.2fV !!", currentVoltage);
        } else if (currentVoltage < 11.5) {
            voltageStatus = String.format("! LOW: %.2fV !", currentVoltage);
        } else {
            voltageStatus = String.format("%.2fV", currentVoltage);
        }
        telemetry.addData("Battery", voltageStatus);

        // Intake status
        double intakePower = intakeTransfer.getIntakePower();
        String intakeStatus = "STOPPED";
        if (intakePower > 0.1) {
            intakeStatus = "INTAKE";
        } else if (intakePower < -0.1) {
            intakeStatus = "EJECT";
        }
        telemetry.addData("Intake", intakeStatus);
        telemetry.addLine();

        // Velocity info
        telemetry.addLine("--- VELOCITY (Motor 1) ---");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Avg RPM (smooth)", "%.0f", avgRPM);
        telemetry.addData("Error", "%.0f RPM", targetRPM - avgRPM);
        telemetry.addData("Error %", "%.1f%%", ((targetRPM - avgRPM) / targetRPM) * 100);
        telemetry.addLine();

        // Power output
        telemetry.addLine("--- OUTPUT ---");
        telemetry.addData("Motor Power", "%.4f (%.1f%%)", lastPower, lastPower * 100);
        telemetry.addLine();

        // PIDF Components
        telemetry.addLine("--- PIDF COMPONENTS ---");
        telemetry.addData("F (feedforward)", "%.4f", fComponent);
        telemetry.addData("P (proportional)", "%.4f", pComponent);
        telemetry.addData("I (integral)", "%.4f", iComponent);
        telemetry.addData("D (derivative)", "%.4f", dComponent);
        telemetry.addLine();

        // Gains
        telemetry.addLine("--- GAINS ---");
        telemetry.addData("kF", "%.6f", kF);
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kI", "%.6f", kI);
        telemetry.addData("kD", "%.6f", kD);
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Y: Toggle | X: Reset PID | B: Save");
        telemetry.addLine("DPAD UP/DN: ±50 RPM | Stick: Fine");
        telemetry.addLine("RT: Intake | LT: Eject");
        telemetry.addLine();
        telemetry.addLine("GAIN TUNING:");
        telemetry.addLine("LB+DPAD: kP | RB+DPAD: kF");
        telemetry.addLine("LB+RB+DPAD: kI | A+DPAD: kD");
        telemetry.addLine();
        telemetry.addLine("NOTE: PIDF auto-compensates for voltage");

        telemetry.update();
    }
}

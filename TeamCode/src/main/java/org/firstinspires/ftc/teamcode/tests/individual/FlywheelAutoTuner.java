package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TuningController;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;

/**
 * *** AUTOMATED FLYWHEEL PIDF TUNER ***
 *
 * This tuner automatically cycles through different speeds to test your PIDF gains.
 * Unlike the manual tuner, this runs a predetermined sequence and logs performance data.
 *
 * HOW TO USE:
 * ===========
 * 1. Make sure your PIDF gains are set in RobotConstants.java
 * 2. Run this OpMode
 * 3. Press START (A button) to begin the automated sequence
 * 4. Watch the telemetry for performance metrics
 * 5. The sequence will automatically cycle through:
 *    - Ramping up from min to max speed
 *    - Coasting at max speed
 *    - Ramping down to min speed
 *    - Coasting at min speed
 *    - Random speed changes
 *    - Rest period
 *
 * CONTROLS:
 * =========
 * - A: Start/Stop the automated sequence
 * - X: Reset PID state
 * - Y: Manual emergency stop
 *
 * WHAT TO LOOK FOR:
 * =================
 * - Steady-state error should be < ±30 RPM
 * - Overshoot should be < 5%
 * - Recovery time should be < 200ms
 * - Motors should stay synchronized (< 100 RPM difference)
 *
 * TUNING PARAMETERS:
 * ==================
 * Adjust these in TuningController.java:
 * - TESTING_MAX_SPEED: Maximum test speed (default: 90% of motor max)
 * - TESTING_MIN_SPEED: Minimum test speed (default: 30% of motor max)
 * - State durations: How long each test phase lasts
 */
@TeleOp(name = "Tuner: Flywheel Auto PIDF", group = "Tuning")
public class FlywheelAutoTuner extends LinearOpMode {

    // Hardware
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    private VoltageSensor voltageSensor;
    private com.qualcomm.robotcore.hardware.Servo hoodServo;

    // Tuning controller
    private TuningController tuningController;

    // PIDF Gains from RobotConstants
    private double kP = RobotConstants.FLYWHEEL_KP;
    private double kI = RobotConstants.FLYWHEEL_KI;
    private double kD = RobotConstants.FLYWHEEL_KD;
    private double kF = RobotConstants.FLYWHEEL_KF;

    // Voltage monitoring
    private static final double CRITICAL_VOLTAGE = 10.5;
    private double currentVoltage = 13.0;

    // Control state
    private boolean sequenceRunning = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastY = false;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Velocity measurement (BOTH motors)
    private int lastPosition1 = 0;
    private int lastPosition2 = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double currentRPM1 = 0.0;
    private double currentRPM2 = 0.0;
    private double currentRPM = 0.0;  // Average of both motors
    private double avgRPM = 0.0;

    // Velocity smoothing
    private static final int VELOCITY_SAMPLES = 5;
    private double[] velocitySamples = new double[VELOCITY_SAMPLES];
    private int velocityIndex = 0;

    // Telemetry tracking
    private double lastPower = 0.0;
    private double pComponent = 0.0;
    private double iComponent = 0.0;
    private double dComponent = 0.0;
    private double fComponent = 0.0;

    // Performance metrics
    private double maxError = 0.0;
    private double avgError = 0.0;
    private double errorSum = 0.0;
    private int errorSamples = 0;
    private double maxOvershoot = 0.0;

    // Target tracking
    private double targetRPM = 0.0;
    private double lastTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        initializeHardware();
        tuningController = new TuningController();

        telemetry.addLine("=== AUTOMATED FLYWHEEL PIDF TUNER ===");
        telemetry.addLine();
        telemetry.addLine("This will automatically test your PIDF gains");
        telemetry.addLine("by cycling through different speeds.");
        telemetry.addLine();
        telemetry.addLine("CURRENT GAINS:");
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kI", "%.6f", kI);
        telemetry.addData("kD", "%.6f", kD);
        telemetry.addData("kF", "%.6f", kF);
        telemetry.addLine();
        telemetry.addData("Hood", hoodServo != null ? "LOCKED at " + RobotConstants.HOOD_DEFAULT_POSITION : "NOT FOUND");
        telemetry.addData("Battery", "%.2fV", currentVoltage);
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("- Press A to START/STOP sequence");
        telemetry.addLine("- Press X to reset PID");
        telemetry.addLine("- Press Y for emergency stop");
        telemetry.addLine();
        telemetry.addLine("Press START when ready...");
        telemetry.update();

        waitForStart();

        pidTimer.reset();
        velocityTimer.reset();

        while (opModeIsActive()) {
            handleControls();
            measureVelocity();

            if (sequenceRunning) {
                // Get target velocity from tuning controller (in ticks/sec)
                double targetTicksPerSec = tuningController.update();

                // Convert to RPM for our system
                targetRPM = ticksPerSecondToRPM(targetTicksPerSec);

                // Calculate and apply PIDF
                double power = calculatePIDFPower();
                setFlywheelPower(power);

                // Track performance metrics
                updateMetrics();
            } else {
                setFlywheelPower(0.0);
                resetPID();
            }

            displayTelemetry();
        }

        // Cleanup
        setFlywheelPower(0.0);
    }

    // ========================================
    // INITIALIZATION
    // ========================================

    private void initializeHardware() {
        // Initialize flywheel motors
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR);
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR_2);

        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastPosition1 = flywheelMotor1.getCurrentPosition();
        lastPosition2 = flywheelMotor2.getCurrentPosition();

        // Initialize and lock hood servo at default position for consistent testing
        try {
            hoodServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, HardwareConfig.HOOD_SERVO);
            hoodServo.setPosition(RobotConstants.HOOD_DEFAULT_POSITION);
        } catch (Exception e) {
            hoodServo = null; // Hood servo not configured - that's OK
        }

        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }
    }

    // ========================================
    // VELOCITY MEASUREMENT (BOTH MOTORS)
    // ========================================

    private void measureVelocity() {
        double dt = velocityTimer.seconds();
        if (dt < 0.02) return; // Sample at ~50Hz

        int currentPos1 = flywheelMotor1.getCurrentPosition();
        int currentPos2 = flywheelMotor2.getCurrentPosition();

        // Calculate velocity in RPM for BOTH motors (28 ticks per revolution)
        // Use ABSOLUTE VALUE because motors may be mounted opposite each other
        currentRPM1 = Math.abs(((currentPos1 - lastPosition1) / dt) * (60.0 / 28.0));
        currentRPM2 = Math.abs(((currentPos2 - lastPosition2) / dt) * (60.0 / 28.0));

        // Average the two motor velocities for PID control
        currentRPM = (currentRPM1 + currentRPM2) / 2.0;

        // Smooth using moving average
        velocitySamples[velocityIndex] = currentRPM;
        velocityIndex = (velocityIndex + 1) % VELOCITY_SAMPLES;

        double sum = 0;
        for (double sample : velocitySamples) {
            sum += sample;
        }
        avgRPM = sum / VELOCITY_SAMPLES;

        lastPosition1 = currentPos1;
        lastPosition2 = currentPos2;
        velocityTimer.reset();
    }

    // ========================================
    // PIDF CONTROLLER
    // ========================================

    private double calculatePIDFPower() {
        // Update voltage
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }

        // Emergency shutoff
        if (currentVoltage < CRITICAL_VOLTAGE) {
            resetPID();
            return 0.0;
        }

        double error = targetRPM - avgRPM;
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt > 1.0) dt = 0.02; // Prevent huge jumps

        // Feedforward
        fComponent = kF * targetRPM;

        // Proportional
        pComponent = kP * error;

        // Integral with anti-windup
        integral += error * dt;
        integral = Math.max(-1000, Math.min(1000, integral));
        iComponent = kI * integral;

        // Derivative
        double derivative = (error - lastError) / dt;
        dComponent = kD * derivative;

        lastError = error;

        // Combine all components
        double power = fComponent + pComponent + iComponent + dComponent;

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
        // Toggle sequence
        if (gamepad1.a && !lastA) {
            sequenceRunning = !sequenceRunning;
            if (sequenceRunning) {
                tuningController.start();
                resetMetrics();
            } else {
                resetPID();
            }
        }
        lastA = gamepad1.a;

        // Reset PID
        if (gamepad1.x && !lastX) {
            resetPID();
        }
        lastX = gamepad1.x;

        // Emergency stop
        if (gamepad1.y && !lastY) {
            sequenceRunning = false;
            resetPID();
        }
        lastY = gamepad1.y;
    }

    // ========================================
    // MOTOR CONTROL
    // ========================================

    private void setFlywheelPower(double power) {
        flywheelMotor1.setPower(power);
        flywheelMotor2.setPower(power);
    }

    // ========================================
    // PERFORMANCE METRICS
    // ========================================

    private void updateMetrics() {
        double error = Math.abs(targetRPM - avgRPM);

        // Track max error
        if (error > maxError) {
            maxError = error;
        }

        // Track average error
        errorSum += error;
        errorSamples++;
        avgError = errorSum / errorSamples;

        // Track overshoot when target changes
        if (Math.abs(targetRPM - lastTargetRPM) > 100) {
            // Target changed significantly, watch for overshoot
            lastTargetRPM = targetRPM;
        }

        if (avgRPM > targetRPM) {
            double overshoot = ((avgRPM - targetRPM) / targetRPM) * 100;
            if (overshoot > maxOvershoot) {
                maxOvershoot = overshoot;
            }
        }
    }

    private void resetMetrics() {
        maxError = 0.0;
        avgError = 0.0;
        errorSum = 0.0;
        errorSamples = 0;
        maxOvershoot = 0.0;
        lastTargetRPM = 0.0;
    }

    // ========================================
    // UTILITY
    // ========================================

    private double ticksPerSecondToRPM(double ticksPerSec) {
        // TuningController uses ticks/sec, we use RPM
        // 28 ticks per revolution
        return (ticksPerSec / 28.0) * 60.0;
    }

    // ========================================
    // TELEMETRY
    // ========================================

    private void displayTelemetry() {
        telemetry.addLine("=== AUTOMATED FLYWHEEL TUNER ===");
        telemetry.addData("Sequence", sequenceRunning ? "RUNNING" : "STOPPED");
        if (sequenceRunning && tuningController != null) {
            telemetry.addData("Current State", tuningController.getCurrentStateName());
            telemetry.addData("State Time", "%.1fs", tuningController.getCurrentStateTime());
        }
        telemetry.addData("Battery", "%.2fV", currentVoltage);
        telemetry.addLine();

        // Current status
        telemetry.addLine("--- CURRENT STATE ---");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Motor 1 RPM", "%.0f", currentRPM1);
        telemetry.addData("Motor 2 RPM", "%.0f", currentRPM2);
        telemetry.addData("Average RPM", "%.0f", currentRPM);
        telemetry.addData("Smoothed RPM", "%.0f", avgRPM);

        // Motor sync check
        double motorDiff = Math.abs(currentRPM1 - currentRPM2);
        String syncStatus;
        if (motorDiff < 50) {
            syncStatus = "SYNCED ✓";
        } else if (motorDiff < 100) {
            syncStatus = "SLIGHTLY OFF";
        } else {
            syncStatus = "OUT OF SYNC!";
        }
        telemetry.addData("Motor Sync", "%.0f RPM diff (%s)", motorDiff, syncStatus);
        telemetry.addLine();

        // Error tracking
        double currentError = targetRPM - avgRPM;
        telemetry.addLine("--- ERROR TRACKING ---");
        telemetry.addData("Current Error", "%.0f RPM", currentError);
        telemetry.addData("Max Error", "%.0f RPM", maxError);
        telemetry.addData("Avg Error", "%.0f RPM", avgError);
        telemetry.addData("Max Overshoot", "%.1f%%", maxOvershoot);
        telemetry.addLine();

        // Performance assessment
        String performance;
        if (sequenceRunning) {
            if (avgError < 30 && maxOvershoot < 5) {
                performance = "EXCELLENT ✓✓✓";
            } else if (avgError < 50 && maxOvershoot < 10) {
                performance = "GOOD ✓";
            } else if (avgError < 100) {
                performance = "NEEDS TUNING";
            } else {
                performance = "POOR - RETUNE!";
            }
        } else {
            performance = "NOT RUNNING";
        }
        telemetry.addData("Performance", performance);
        telemetry.addLine();

        // PIDF components
        telemetry.addLine("--- PIDF OUTPUT ---");
        telemetry.addData("Motor Power", "%.4f (%.1f%%)", lastPower, lastPower * 100);
        telemetry.addData("F (feedforward)", "%.4f", fComponent);
        telemetry.addData("P (proportional)", "%.4f", pComponent);
        telemetry.addData("I (integral)", "%.4f", iComponent);
        telemetry.addData("D (derivative)", "%.4f", dComponent);
        telemetry.addLine();

        // Gains being tested
        telemetry.addLine("--- CURRENT GAINS ---");
        telemetry.addData("kF", "%.6f", kF);
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kI", "%.6f", kI);
        telemetry.addData("kD", "%.6f", kD);
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("A: Start/Stop | X: Reset PID | Y: E-Stop");

        telemetry.update();
    }
}

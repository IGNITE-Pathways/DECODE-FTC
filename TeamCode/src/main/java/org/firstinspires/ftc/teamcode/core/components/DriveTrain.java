package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

public class DriveTrain {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private Telemetry telemetry;

    // ==================== DRIVER COMFORT SETTINGS ====================

    // Deadzone - ignore small joystick movements
    private static final double DEADZONE = 0.05;

    // Input curve exponent (higher = more precision at low speeds)
    // 1.0 = linear, 2.0 = squared, 3.0 = cubed
    private double inputCurveExponent = 2.0;

    // Speed multipliers for different modes
    public enum SpeedMode {
        SLOW(0.35),      // Precise movements
        NORMAL(0.7),     // Default driving
        TURBO(1.0);      // Full speed

        public final double multiplier;
        SpeedMode(double multiplier) {
            this.multiplier = multiplier;
        }
    }
    private SpeedMode currentSpeedMode = SpeedMode.NORMAL;

    // Rotation sensitivity (reduce for easier straight driving)
    private double rotationSensitivity = 0.7;

    // Input smoothing (ramping)
    private boolean smoothingEnabled = true;
    private static final double SMOOTHING_FACTOR = 0.15;  // Lower = smoother (0.1-0.3 recommended)

    // Smoothed input values
    private double smoothedForward = 0;
    private double smoothedRight = 0;
    private double smoothedRotate = 0;

    // Current motor powers (for telemetry)
    private double currentFL = 0, currentFR = 0, currentBL = 0, currentBR = 0;

    // ==================== BATTERY MANAGEMENT ====================
    private VoltageSensor voltageSensor;
    private boolean batteryCompensationEnabled = true;

    // Voltage constants
    private static final double NOMINAL_VOLTAGE = 12.5;
    private static final double MIN_VOLTAGE = 10.0;
    private static final double MAX_COMPENSATION = 1.25;
    private static final double MIN_COMPENSATION = 0.85;

    // Voltage smoothing
    private double smoothedVoltage = NOMINAL_VOLTAGE;
    private static final double VOLTAGE_SMOOTHING_ALPHA = 0.1;

    // Current state
    private double currentVoltage = NOMINAL_VOLTAGE;
    private double compensationFactor = 1.0;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_FRONT_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_FRONT_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_BACK_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_BACK_MOTOR);

        // Correct motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to BRAKE for better control
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize voltage sensor for battery management
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        if (voltageSensor != null) {
            smoothedVoltage = voltageSensor.getVoltage();
            currentVoltage = smoothedVoltage;
        }

        telemetry.addData("Status", "DriveTrain Initialized");
        telemetry.addData("Speed Mode", currentSpeedMode.name());
        telemetry.addData("Battery", "%.2f V", currentVoltage);
        telemetry.update();
    }

    public void update(double forward, double right, double rotate) {
        // Update battery compensation
        updateBatteryCompensation();

        // Apply deadzone
        forward = applyDeadzone(forward);
        right = applyDeadzone(right);
        rotate = applyDeadzone(rotate);

        // Apply input curve for precision control
        forward = applyInputCurve(forward);
        right = applyInputCurve(right);
        rotate = applyInputCurve(rotate);

        // Apply rotation sensitivity
        rotate *= rotationSensitivity;

        // Apply speed mode multiplier
        forward *= currentSpeedMode.multiplier;
        right *= currentSpeedMode.multiplier;
        rotate *= currentSpeedMode.multiplier;

        // Apply input smoothing (ramping)
        if (smoothingEnabled) {
            smoothedForward = smoothedForward + (forward - smoothedForward) * SMOOTHING_FACTOR;
            smoothedRight = smoothedRight + (right - smoothedRight) * SMOOTHING_FACTOR;
            smoothedRotate = smoothedRotate + (rotate - smoothedRotate) * SMOOTHING_FACTOR;

            forward = smoothedForward;
            right = smoothedRight;
            rotate = smoothedRotate;
        }

        // Mecanum wheel math
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        // Normalize
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Apply battery compensation
        if (batteryCompensationEnabled) {
            frontLeftPower *= compensationFactor;
            frontRightPower *= compensationFactor;
            backLeftPower *= compensationFactor;
            backRightPower *= compensationFactor;

            // Clamp to valid range after compensation
            frontLeftPower = clamp(frontLeftPower, -1.0, 1.0);
            frontRightPower = clamp(frontRightPower, -1.0, 1.0);
            backLeftPower = clamp(backLeftPower, -1.0, 1.0);
            backRightPower = clamp(backRightPower, -1.0, 1.0);
        }

        // Store for telemetry
        currentFL = frontLeftPower;
        currentFR = frontRightPower;
        currentBL = backLeftPower;
        currentBR = backRightPower;

        // Apply powers
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    // ==================== INPUT PROCESSING ====================

    /**
     * Apply deadzone to input - ignore small movements
     */
    private double applyDeadzone(double input) {
        if (Math.abs(input) < DEADZONE) {
            return 0;
        }
        // Scale input so it starts from 0 after deadzone
        double sign = Math.signum(input);
        return sign * (Math.abs(input) - DEADZONE) / (1.0 - DEADZONE);
    }

    /**
     * Apply input curve for better precision at low speeds
     * Uses power function: output = sign(input) * |input|^exponent
     */
    private double applyInputCurve(double input) {
        double sign = Math.signum(input);
        return sign * Math.pow(Math.abs(input), inputCurveExponent);
    }

    // ==================== SPEED MODE CONTROLS (TOGGLE) ====================

    // Toggle state tracking
    private boolean slowModeActive = false;
    private boolean turboModeActive = false;

    /**
     * Set the current speed mode
     */
    public void setSpeedMode(SpeedMode mode) {
        this.currentSpeedMode = mode;
    }

    /**
     * Get the current speed mode
     */
    public SpeedMode getSpeedMode() {
        return currentSpeedMode;
    }

    /**
     * Cycle to next speed mode
     */
    public void cycleSpeedMode() {
        SpeedMode[] modes = SpeedMode.values();
        int nextIndex = (currentSpeedMode.ordinal() + 1) % modes.length;
        currentSpeedMode = modes[nextIndex];
    }

    /**
     * Toggle slow mode ON/OFF
     * Call this on button PRESS (use edge detection in TeleOp)
     */
    public void toggleSlowMode() {
        slowModeActive = !slowModeActive;
        turboModeActive = false;  // Turn off turbo if slow is toggled
        updateSpeedModeFromToggles();
    }

    /**
     * Toggle turbo mode ON/OFF
     * Call this on button PRESS (use edge detection in TeleOp)
     */
    public void toggleTurboMode() {
        turboModeActive = !turboModeActive;
        slowModeActive = false;  // Turn off slow if turbo is toggled
        updateSpeedModeFromToggles();
    }

    /**
     * Update speed mode based on toggle states
     */
    private void updateSpeedModeFromToggles() {
        if (slowModeActive) {
            currentSpeedMode = SpeedMode.SLOW;
        } else if (turboModeActive) {
            currentSpeedMode = SpeedMode.TURBO;
        } else {
            currentSpeedMode = SpeedMode.NORMAL;
        }
    }

    /**
     * Check if slow mode is currently active
     */
    public boolean isSlowModeActive() {
        return slowModeActive;
    }

    /**
     * Check if turbo mode is currently active
     */
    public boolean isTurboModeActive() {
        return turboModeActive;
    }

    /**
     * Reset to normal mode (turns off both toggles)
     */
    public void resetToNormalMode() {
        slowModeActive = false;
        turboModeActive = false;
        currentSpeedMode = SpeedMode.NORMAL;
    }

    // Legacy methods (still work for hold-to-activate if needed)
    public void setSlowMode() {
        currentSpeedMode = SpeedMode.SLOW;
    }

    public void setNormalMode() {
        currentSpeedMode = SpeedMode.NORMAL;
    }

    public void setTurboMode() {
        currentSpeedMode = SpeedMode.TURBO;
    }

    // ==================== DRIVER COMFORT SETTINGS ====================

    /**
     * Set input curve exponent
     * @param exponent 1.0 = linear, 2.0 = squared (recommended), 3.0 = cubed
     */
    public void setInputCurve(double exponent) {
        this.inputCurveExponent = clamp(exponent, 1.0, 3.0);
    }

    /**
     * Get current input curve exponent
     */
    public double getInputCurve() {
        return inputCurveExponent;
    }

    /**
     * Set rotation sensitivity
     * @param sensitivity 0.0 to 1.0 (lower = less sensitive)
     */
    public void setRotationSensitivity(double sensitivity) {
        this.rotationSensitivity = clamp(sensitivity, 0.1, 1.0);
    }

    /**
     * Get current rotation sensitivity
     */
    public double getRotationSensitivity() {
        return rotationSensitivity;
    }

    /**
     * Enable or disable input smoothing
     */
    public void setSmoothingEnabled(boolean enabled) {
        this.smoothingEnabled = enabled;
        if (!enabled) {
            // Reset smoothed values
            smoothedForward = 0;
            smoothedRight = 0;
            smoothedRotate = 0;
        }
    }

    /**
     * Check if smoothing is enabled
     */
    public boolean isSmoothingEnabled() {
        return smoothingEnabled;
    }

    // ==================== BATTERY COMPENSATION ====================

    private void updateBatteryCompensation() {
        if (voltageSensor == null) {
            compensationFactor = 1.0;
            return;
        }

        currentVoltage = voltageSensor.getVoltage();
        smoothedVoltage = VOLTAGE_SMOOTHING_ALPHA * currentVoltage
                        + (1 - VOLTAGE_SMOOTHING_ALPHA) * smoothedVoltage;

        if (smoothedVoltage > MIN_VOLTAGE) {
            compensationFactor = NOMINAL_VOLTAGE / smoothedVoltage;
        } else {
            compensationFactor = NOMINAL_VOLTAGE / MIN_VOLTAGE;
        }

        compensationFactor = clamp(compensationFactor, MIN_COMPENSATION, MAX_COMPENSATION);
    }

    // ==================== BATTERY MANAGEMENT CONTROLS ====================

    public void setBatteryCompensationEnabled(boolean enabled) {
        this.batteryCompensationEnabled = enabled;
    }

    public boolean isBatteryCompensationEnabled() {
        return batteryCompensationEnabled;
    }

    public double getBatteryVoltage() {
        return currentVoltage;
    }

    public double getSmoothedVoltage() {
        return smoothedVoltage;
    }

    public double getCompensationFactor() {
        return compensationFactor;
    }

    public boolean isBatteryLow() {
        return smoothedVoltage < 11.5;
    }

    public boolean isBatteryCritical() {
        return smoothedVoltage < 10.5;
    }

    public int getBatteryPercentage() {
        if (smoothedVoltage >= 12.6) return 100;
        if (smoothedVoltage >= 12.4) return 90;
        if (smoothedVoltage >= 12.2) return 80;
        if (smoothedVoltage >= 12.0) return 70;
        if (smoothedVoltage >= 11.8) return 60;
        if (smoothedVoltage >= 11.6) return 50;
        if (smoothedVoltage >= 11.4) return 40;
        if (smoothedVoltage >= 11.2) return 30;
        if (smoothedVoltage >= 11.0) return 20;
        if (smoothedVoltage >= 10.5) return 10;
        return 5;
    }

    // ==================== AUTONOMOUS CONTROL ====================
    // These methods bypass driver comfort features for precise auto control

    /**
     * Set motor powers directly (for autonomous use)
     * Bypasses deadzone, input curves, and smoothing
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    /**
     * Drive with raw values (for autonomous use)
     * Applies mecanum math but no driver comfort features
     */
    public void driveRaw(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        // Normalize
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                              Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        setMotorPowers(fl, fr, bl, br);
    }

    /**
     * Drive forward/backward (for autonomous)
     * @param power -1.0 to 1.0 (positive = forward)
     */
    public void driveForward(double power) {
        setMotorPowers(power, power, power, power);
    }

    /**
     * Strafe left/right (for autonomous)
     * @param power -1.0 to 1.0 (positive = right)
     */
    public void strafe(double power) {
        setMotorPowers(power, -power, -power, power);
    }

    /**
     * Rotate in place (for autonomous)
     * @param power -1.0 to 1.0 (positive = clockwise)
     */
    public void rotate(double power) {
        setMotorPowers(power, -power, power, -power);
    }

    /**
     * Get motor encoders (for odometry/autonomous)
     */
    public int getLeftFrontEncoder() {
        return frontLeftDrive.getCurrentPosition();
    }

    public int getRightFrontEncoder() {
        return frontRightDrive.getCurrentPosition();
    }

    public int getLeftBackEncoder() {
        return backLeftDrive.getCurrentPosition();
    }

    public int getRightBackEncoder() {
        return backRightDrive.getCurrentPosition();
    }

    /**
     * Reset all encoders
     */
    public void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get direct access to motors (for Pedro Pathing or other libraries)
     */
    public DcMotor getFrontLeftMotor() {
        return frontLeftDrive;
    }

    public DcMotor getFrontRightMotor() {
        return frontRightDrive;
    }

    public DcMotor getBackLeftMotor() {
        return backLeftDrive;
    }

    public DcMotor getBackRightMotor() {
        return backRightDrive;
    }

    // ==================== UTILITY ====================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void stopMotors() {
        smoothedForward = 0;
        smoothedRight = 0;
        smoothedRotate = 0;
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        telemetry.addLine("=== DRIVETRAIN ===");
        telemetry.addData("Speed Mode", currentSpeedMode.name() + " (" + (int)(currentSpeedMode.multiplier * 100) + "%)");
        telemetry.addData("Smoothing", smoothingEnabled ? "ON" : "OFF");
        telemetry.addLine("");
        telemetry.addLine("=== BATTERY ===");
        telemetry.addData("Voltage", "%.2f V (%d%%)", smoothedVoltage, getBatteryPercentage());
        telemetry.addData("Compensation", batteryCompensationEnabled ? String.format("+%.0f%%", (compensationFactor - 1.0) * 100) : "OFF");
        if (isBatteryCritical()) {
            telemetry.addLine("!! BATTERY CRITICAL !!");
        } else if (isBatteryLow()) {
            telemetry.addLine("! Battery Low !");
        }
    }
}

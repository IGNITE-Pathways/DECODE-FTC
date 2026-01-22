package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.ShooterConstants;

public class Launcher {
    // private ProgrammingBoardOTHER board;
    private Servo hoodServo;
    private Telemetry telemetry;
    public DcMotorEx flyWheelMotor = null;
    public DcMotorEx flyWheelMotor2 = null;

    private double flywheelPower = ShooterConstants.FLYWHEEL_DEFAULT_POWER;
    private boolean spinning = false;
    private double hoodPosition = ShooterConstants.HOOD_DEFAULT_POSITION;
    private boolean autoHoodEnabled = false;  // Enable distance-based hood adjustment
    private double lastDistance = -1;         // Last known distance for auto-hood

    // Voltage compensation for open-loop mode only (PIDF handles it naturally)
    private VoltageSensor voltageSensor;
    private boolean voltageCompensationEnabled = true;
    private static final double NOMINAL_VOLTAGE = 12.5;  // Fresh battery voltage
    private static final double MIN_VOLTAGE = 11.0;      // Minimum safe voltage
    private static final double CRITICAL_VOLTAGE = 10.5; // Emergency shutoff for PIDF mode

    // ==================== PIDF VELOCITY CONTROL ====================
    private boolean useVelocityControl = false;  // Toggle between open-loop and closed-loop
    private double targetRPM = 3500;             // Target velocity in RPM (60% of 6000 RPM max)

    // PIDF Gains - TUNED FOR 6000 RPM MOTORS
    // Fine-tune these values using FlywheelPIDFTuner if needed
    private double kP = 0.0004;   // Proportional gain
    private double kI = 0.00003;  // Integral gain
    private double kD = 0.0002;   // Derivative gain
    private double kF = 0.00020;  // Feedforward gain

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Velocity measurement (using only motor 1 encoder)
    private int lastPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double currentRPM = 0.0;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // board = new ProgrammingBoardOTHER();
        // board.initializeComponents(hardwareMap);

        // Hood servo is optional - don't move it at init, let it stay where it is
        try {
            hoodServo = hardwareMap.get(Servo.class, HardwareConfig.HOOD_SERVO);
            // Don't set position at init - manual control only
        } catch (Exception e) {
            hoodServo = null;  // No hood servo configured - that's OK
        }

        flyWheelMotor = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR);
        flyWheelMotor2 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR_2);

        // Both motors spin the same direction for flywheel
        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders for velocity measurement
        flyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastPosition = flyWheelMotor.getCurrentPosition();

        // Get voltage sensor for compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pidTimer.reset();
        velocityTimer.reset();

        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    public void update() {
        if (!spinning) {
            flyWheelMotor.setPower(0);
            flyWheelMotor2.setPower(0);
            resetPID();
            return;
        }

        double power;

        if (useVelocityControl) {
            // Closed-loop velocity control using PIDF
            measureVelocity();
            power = calculatePIDFPower();
        } else {
            // Open-loop power control (legacy mode)
            power = flywheelPower;

            // Apply voltage compensation for consistent flywheel speed
            if (voltageCompensationEnabled && voltageSensor != null) {
                double currentVoltage = voltageSensor.getVoltage();
                double compensation = NOMINAL_VOLTAGE / Math.max(currentVoltage, MIN_VOLTAGE);
                power = Math.min(power * compensation, 1.0);  // Cap at 100%
            }
        }

        flyWheelMotor.setPower(power);
        flyWheelMotor2.setPower(power);
    }

    // ==================== VELOCITY MEASUREMENT ====================

    private void measureVelocity() {
        double dt = velocityTimer.seconds();
        if (dt < 0.02) return; // Sample at ~50Hz

        int currentPos = flyWheelMotor.getCurrentPosition();

        // Calculate velocity in RPM (28 ticks per revolution for REV motors)
        currentRPM = ((currentPos - lastPosition) / dt) * (60.0 / 28.0);

        lastPosition = currentPos;
        velocityTimer.reset();
    }

    // ==================== PIDF CONTROLLER ====================

    private double calculatePIDFPower() {
        // Check voltage for emergency shutoff only
        double currentVoltage = NOMINAL_VOLTAGE;
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }

        // CRITICAL: Emergency shutoff if voltage too low
        if (currentVoltage < CRITICAL_VOLTAGE) {
            resetPID();
            return 0.0;
        }

        double error = targetRPM - currentRPM;
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt > 1.0) dt = 0.02; // Prevent huge jumps

        // Feedforward - base power for target velocity
        double fComponent = kF * targetRPM;

        // Proportional - respond to current error
        double pComponent = kP * error;

        // Integral - eliminate steady-state error
        integral += error * dt;
        integral = Math.max(-1000, Math.min(1000, integral)); // Anti-windup
        double iComponent = kI * integral;

        // Derivative - reduce oscillation
        double derivative = (error - lastError) / dt;
        double dComponent = kD * derivative;

        lastError = error;

        // Combine components
        double power = fComponent + pComponent + iComponent + dComponent;

        // NO voltage compensation for PIDF mode - it naturally compensates!
        // When voltage drops → RPM drops → error increases → PIDF adds more power
        // Adding voltage comp causes double-compensation and worse performance

        // Clamp to valid range (allow full power for PIDF)
        return Math.max(0.0, Math.min(1.0, power));
    }

    private void resetPID() {
        integral = 0.0;
        lastError = 0.0;
        pidTimer.reset();
    }

    // Flywheel methods
    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    public void setPower(double power) {
        this.flywheelPower = power;
    }

    public double getPower() {
        return flywheelPower;
    }

    // Hood methods
    public void setHoodPosition(double position) {
        hoodPosition = Math.max(0.0, Math.min(1.0, position));
        if (hoodServo != null) {
            hoodServo.setPosition(hoodPosition);
        }
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public void adjustHoodPosition(double increment) {
        setHoodPosition(hoodPosition + increment);
    }

    public void incrementHood() {
        adjustHoodPosition(0.05);
    }

    public void decrementHood() {
        adjustHoodPosition(-ShooterConstants.HOOD_INCREMENT);
    }

    // ==================== DISTANCE-BASED HOOD ADJUSTMENT ====================

    /**
     * Enable or disable automatic hood adjustment based on distance.
     * When enabled, updateHoodForDistance() will automatically set hood position.
     */
    public void setAutoHoodEnabled(boolean enabled) {
        this.autoHoodEnabled = enabled;
    }

    public boolean isAutoHoodEnabled() {
        return autoHoodEnabled;
    }

    /**
     * Update hood position based on distance to target.
     * Uses lookup table from ShooterConstants for optimal angle.
     *
     * @param distanceFeet Distance to target in feet (from turret/limelight)
     */
    public void updateHoodForDistance(double distanceFeet) {
        if (!autoHoodEnabled || distanceFeet <= 0) {
            return;
        }

        lastDistance = distanceFeet;
        double optimalPosition = ShooterConstants.getHoodPositionForDistance(distanceFeet);
        setHoodPosition(optimalPosition);
    }

    /**
     * Update both hood and flywheel power based on distance.
     * Use this for fully automatic shooting adjustments.
     *
     * @param distanceFeet Distance to target in feet
     */
    public void updateForDistance(double distanceFeet) {
        if (distanceFeet <= 0) {
            return;
        }

        lastDistance = distanceFeet;

        if (autoHoodEnabled) {
            double optimalHood = ShooterConstants.getHoodPositionForDistance(distanceFeet);
            setHoodPosition(optimalHood);
        }

        double optimalPower = ShooterConstants.getFlywheelPowerForDistance(distanceFeet);
        setPower(optimalPower);
    }

    /**
     * Get the last known distance used for auto-adjustment.
     */
    public double getLastDistance() {
        return lastDistance;
    }

    // ==================== VOLTAGE COMPENSATION ====================

    /**
     * Enable or disable voltage compensation for consistent flywheel speed.
     * When enabled, flywheel power is automatically adjusted based on battery voltage.
     */
    public void setVoltageCompensationEnabled(boolean enabled) {
        this.voltageCompensationEnabled = enabled;
    }

    public boolean isVoltageCompensationEnabled() {
        return voltageCompensationEnabled;
    }

    /**
     * Get the current battery voltage.
     * @return Current voltage, or -1 if sensor unavailable
     */
    public double getCurrentVoltage() {
        if (voltageSensor != null) {
            return voltageSensor.getVoltage();
        }
        return -1;
    }

    // ==================== PIDF VELOCITY CONTROL ====================

    /**
     * Enable closed-loop velocity control (PIDF) instead of open-loop power.
     * This provides consistent RPM and fast recovery after shots.
     * Requires tuning PIDF gains first using FlywheelPIDFTuner.
     */
    public void setVelocityControlEnabled(boolean enabled) {
        this.useVelocityControl = enabled;
        if (enabled) {
            resetPID();
        }
    }

    /**
     * Check if velocity control is enabled.
     */
    public boolean isVelocityControlEnabled() {
        return useVelocityControl;
    }

    /**
     * Set target RPM for velocity control.
     * Only used when velocity control is enabled.
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /**
     * Get target RPM for velocity control.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Get current measured RPM.
     */
    public double getCurrentRPM() {
        return currentRPM;
    }

    /**
     * Set PIDF gains.
     * Use FlywheelPIDFTuner to find optimal values for your robot.
     *
     * @param kP Proportional gain (responds to error)
     * @param kI Integral gain (eliminates steady-state error)
     * @param kD Derivative gain (reduces oscillation)
     * @param kF Feedforward gain (base power for target velocity)
     */
    public void setPIDFGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Get PIDF gains as array [kP, kI, kD, kF].
     */
    public double[] getPIDFGains() {
        return new double[]{kP, kI, kD, kF};
    }
}


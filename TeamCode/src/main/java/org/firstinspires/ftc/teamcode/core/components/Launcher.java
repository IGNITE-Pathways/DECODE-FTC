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
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;

public class Launcher {
    // private ProgrammingBoardOTHER board;
    private Servo hoodServo;
    private Telemetry telemetry;
    public DcMotorEx flyWheelMotor = null;
    public DcMotorEx flyWheelMotor2 = null;

    private double flywheelPower = RobotConstants.FLYWHEEL_DEFAULT_POWER;
    private boolean spinning = false;
    private double hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
    private boolean autoHoodEnabled = false;  // Enable distance-based hood adjustment
    private double lastDistance = -1;         // Last known distance for auto-hood

    // Voltage compensation for open-loop mode only (PIDF handles it naturally)
    private VoltageSensor voltageSensor;
    private boolean voltageCompensationEnabled = true;
    private static final double NOMINAL_VOLTAGE = 12.5;  // Fresh battery voltage
    private static final double MIN_VOLTAGE = 11.0;      // Minimum safe voltage
    private static final double CRITICAL_VOLTAGE = 10.5; // Emergency shutoff for PIDF mode

    // ==================== PIDF VELOCITY CONTROL ====================
    // All PIDF values come from RobotConstants - update there after tuning!
    private boolean useVelocityControl = RobotConstants.USE_VELOCITY_CONTROL;
    private double targetRPM = RobotConstants.DEFAULT_TARGET_RPM;

    // PIDF Gains - loaded from RobotConstants (tune using FlywheelPIDFTuner)
    private double kP = RobotConstants.FLYWHEEL_KP;
    private double kI = RobotConstants.FLYWHEEL_KI;
    private double kD = RobotConstants.FLYWHEEL_KD;
    private double kF = RobotConstants.FLYWHEEL_KF;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Velocity measurement (using both motor encoders for accuracy)
    private int lastPosition1 = 0;
    private int lastPosition2 = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double currentRPM = 0.0;
    private double currentRPM1 = 0.0;  // Individual motor 1 RPM
    private double currentRPM2 = 0.0;  // Individual motor 2 RPM

    // Startup boost for faster spinup (settings loaded from RobotConstants)
    private ElapsedTime spinupTimer = new ElapsedTime();
    private boolean justStarted = false;

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

        lastPosition1 = flyWheelMotor.getCurrentPosition();
        lastPosition2 = flyWheelMotor2.getCurrentPosition();

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
            justStarted = false;
            return;
        }

        // Check if we just started spinning
        if (!justStarted) {
            justStarted = true;
            spinupTimer.reset();
        }

        double power;

        if (useVelocityControl) {
            // Closed-loop velocity control using PIDF
            measureVelocity();

            // Apply startup boost for faster spinup
            boolean inBoostPhase = spinupTimer.seconds() < RobotConstants.SPINUP_BOOST_DURATION;
            boolean belowThreshold = currentRPM < (targetRPM * RobotConstants.SPINUP_RPM_THRESHOLD);

            if (inBoostPhase && belowThreshold) {
                // Startup boost: apply high power to quickly reach target speed
                power = RobotConstants.SPINUP_BOOST_POWER;
            } else {
                // Normal PIDF control
                power = calculatePIDFPower();
            }
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

        int currentPos1 = flyWheelMotor.getCurrentPosition();
        int currentPos2 = flyWheelMotor2.getCurrentPosition();

        // Calculate velocity in RPM for both motors (28 ticks per revolution for REV motors)
        // Use ABSOLUTE VALUE because motors may be mounted opposite each other
        currentRPM1 = Math.abs(((currentPos1 - lastPosition1) / dt) * (60.0 / 28.0));
        currentRPM2 = Math.abs(((currentPos2 - lastPosition2) / dt) * (60.0 / 28.0));

        // Average both motors for PIDF feedback (more accurate than using only one)
        currentRPM = (currentRPM1 + currentRPM2) / 2.0;

        lastPosition1 = currentPos1;
        lastPosition2 = currentPos2;
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
        adjustHoodPosition(-RobotConstants.HOOD_INCREMENT);
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
     * Uses lookup table from RobotConstants for optimal angle.
     *
     * @param distanceFeet Distance to target in feet (from turret/limelight)
     */
    public void updateHoodForDistance(double distanceFeet) {
        if (!autoHoodEnabled || distanceFeet <= 0) {
            return;
        }

        lastDistance = distanceFeet;
        double optimalPosition = RobotConstants.getHoodPositionForDistance(distanceFeet);
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
            double optimalHood = RobotConstants.getHoodPositionForDistance(distanceFeet);
            setHoodPosition(optimalHood);
        }

        double optimalPower = RobotConstants.getFlywheelPowerForDistance(distanceFeet);
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
     * Get current measured RPM (average of both motors).
     */
    public double getCurrentRPM() {
        return currentRPM;
    }

    /**
     * Get current RPM of motor 1.
     */
    public double getMotor1RPM() {
        return currentRPM1;
    }

    /**
     * Get current RPM of motor 2.
     */
    public double getMotor2RPM() {
        return currentRPM2;
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

    /**
     * Check if flywheel is in startup boost phase.
     * @return true if currently using boost power for faster spinup
     */
    public boolean isInBoostPhase() {
        if (!spinning || !useVelocityControl) {
            return false;
        }

        boolean inBoostPhase = spinupTimer.seconds() < RobotConstants.SPINUP_BOOST_DURATION;
        boolean belowThreshold = currentRPM < (targetRPM * RobotConstants.SPINUP_RPM_THRESHOLD);

        return inBoostPhase && belowThreshold;
    }
}


package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.ShooterConstants;

public class Launcher {
    // private ProgrammingBoardOTHER board;
    private Servo hoodServo;
    private Telemetry telemetry;
    public DcMotor flyWheelMotor = null;
    public DcMotor flyWheelMotor2 = null;

    private double flywheelPower = ShooterConstants.FLYWHEEL_DEFAULT_POWER;
    private boolean spinning = false;
    private double hoodPosition = ShooterConstants.HOOD_DEFAULT_POSITION;
    private boolean autoHoodEnabled = false;  // Enable distance-based hood adjustment
    private double lastDistance = -1;         // Last known distance for auto-hood

    // Voltage compensation for consistent flywheel performance
    private VoltageSensor voltageSensor;
    private boolean voltageCompensationEnabled = true;
    private static final double NOMINAL_VOLTAGE = 12.5;  // Fresh battery voltage
    private static final double MIN_VOLTAGE = 11.0;      // Minimum safe voltage

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

        flyWheelMotor = hardwareMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR);
        flyWheelMotor2 = hardwareMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR_2);

        // Both motors spin the same direction for flywheel
        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Get voltage sensor for compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    public void update() {
        double power = spinning ? flywheelPower : 0;

        // Apply voltage compensation for consistent flywheel speed
        if (spinning && voltageCompensationEnabled && voltageSensor != null) {
            double currentVoltage = voltageSensor.getVoltage();
            double compensation = NOMINAL_VOLTAGE / Math.max(currentVoltage, MIN_VOLTAGE);
            power = Math.min(power * compensation, 1.0);  // Cap at 100%
        }

        if (flyWheelMotor != null) {
            flyWheelMotor.setPower(power);
        }
        if (flyWheelMotor2 != null) {
            flyWheelMotor2.setPower(power);
        }
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
}


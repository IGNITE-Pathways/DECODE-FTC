package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

/**
 * Intake component for FTC robot.
 * Controls a continuous rotation servo for ball intake/ejection.
 * Supports power ramping, state tracking, and safety features.
 */
public class Intake {
    /**
     * State enum for intake operation
     */
    public enum State {
        IDLE,
        RUNNING_FORWARD,
        RUNNING_REVERSE,
        ERROR
    }
    
    // Hardware
    private CRServo intakeServo;
    private Telemetry telemetry;
    
    // State tracking
    private boolean initialized = false;
    private State state = State.IDLE;
    private double currentPower = 0.0;
    private double targetPower = 0.0;
    private boolean running = false;
    
    // Power ramping
    private boolean rampingEnabled = false;
    private double rampRate = IntakeConstants.DEFAULT_RAMP_RATE;
    
    // Safety
    private long startTime = 0;
    private boolean safetyTimeoutEnabled = false;
    
    // Telemetry control
    private boolean telemetryEnabled = true;
    
    /**
     * Initialize the intake component.
     * Sets up hardware and validates configuration.
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry instance (can be null)
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initialized = false;
        state = State.IDLE;
        
        try {
            intakeServo = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO);
            
            if (intakeServo != null) {
                intakeServo.setPower(0.0);
                initialized = true;
                
                if (telemetry != null) {
                    telemetry.addLine("Intake Component Initialized");
                    telemetry.update();
                }
            } else {
                state = State.ERROR;
                if (telemetry != null) {
                    telemetry.addLine("ERROR: Intake servo not found in hardware map!");
                    telemetry.update();
                }
            }
        } catch (Exception e) {
            state = State.ERROR;
            initialized = false;
            if (telemetry != null) {
                telemetry.addLine("ERROR: Intake initialization failed: " + e.getMessage());
                telemetry.update();
            }
        }
    }
    
    /**
     * Set the power level of the intake servo.
     * Power is clamped to [-1.0, 1.0] range.
     * 
     * @param power Power level (-1.0 to 1.0, positive = forward)
     */
    public void setPower(double power) {
        if (!initialized || intakeServo == null) {
            return;
        }
        
        // Clamp power to valid range
        targetPower = Math.max(-1.0, Math.min(1.0, power));
        
        // If ramping is disabled, set power immediately (maintains backward compatibility)
        if (!rampingEnabled) {
            currentPower = targetPower;
            applyPower();
            // Update running state immediately to match original behavior
            running = Math.abs(currentPower) > IntakeConstants.POWER_THRESHOLD;
        }
        // Otherwise, ramping will be handled in update()
    }
    
    /**
     * Start intake at default forward power.
     */
    public void start() {
        start(IntakeConstants.DEFAULT_FORWARD_POWER);
    }
    
    /**
     * Start intake at specified power.
     * 
     * @param power Power level (-1.0 to 1.0, positive = forward)
     */
    public void start(double power) {
        setPower(power);
        if (power > IntakeConstants.POWER_THRESHOLD) {
            state = State.RUNNING_FORWARD;
            startSafetyTimer();
        } else if (power < -IntakeConstants.POWER_THRESHOLD) {
            state = State.RUNNING_REVERSE;
            startSafetyTimer();
        } else {
            state = State.IDLE;
            stopSafetyTimer();
        }
    }
    
    /**
     * Stop the intake immediately.
     */
    public void stop() {
        setPower(0.0);
        state = State.IDLE;
        stopSafetyTimer();
        // Disable ramping for immediate stop
        boolean wasRamping = rampingEnabled;
        rampingEnabled = false;
        currentPower = 0.0;
        applyPower();
        rampingEnabled = wasRamping;
    }
    
    /**
     * Reverse intake at default reverse power (eject).
     */
    public void reverse() {
        start(IntakeConstants.DEFAULT_REVERSE_POWER);
    }
    
    /**
     * Get current power level.
     * 
     * @return Current power level (-1.0 to 1.0)
     */
    public double getPower() {
        return currentPower;
    }
    
    /**
     * Get target power level (for ramping).
     * 
     * @return Target power level (-1.0 to 1.0)
     */
    public double getTargetPower() {
        return targetPower;
    }
    
    /**
     * Check if intake is currently running.
     * 
     * @return true if intake power is above threshold
     */
    public boolean isRunning() {
        return running;
    }
    
    /**
     * Check if intake is properly initialized.
     * 
     * @return true if initialization was successful
     */
    public boolean isInitialized() {
        return initialized;
    }
    
    /**
     * Get current intake state.
     * 
     * @return Current state enum
     */
    public State getState() {
        return state;
    }
    
    /**
     * Enable or disable telemetry updates.
     * 
     * @param enabled true to enable telemetry, false to disable
     */
    public void setTelemetryEnabled(boolean enabled) {
        telemetryEnabled = enabled;
    }
    
    /**
     * Check if telemetry is enabled.
     * 
     * @return true if telemetry updates are enabled
     */
    public boolean isTelemetryEnabled() {
        return telemetryEnabled;
    }
    
    /**
     * Ramp to target power over time.
     * 
     * @param targetPower Target power level (-1.0 to 1.0)
     * @param rampRate Power change per update (typically 0.01 to 0.1)
     */
    public void rampToPower(double targetPower, double rampRate) {
        this.targetPower = Math.max(-1.0, Math.min(1.0, targetPower));
        this.rampRate = Math.abs(rampRate);
        rampingEnabled = true;
        
        if (this.targetPower > IntakeConstants.POWER_THRESHOLD) {
            state = State.RUNNING_FORWARD;
            startSafetyTimer();
        } else if (this.targetPower < -IntakeConstants.POWER_THRESHOLD) {
            state = State.RUNNING_REVERSE;
            startSafetyTimer();
        } else {
            state = State.IDLE;
            stopSafetyTimer();
        }
    }
    
    /**
     * Enable or disable power ramping.
     * 
     * @param enabled true to enable ramping, false to disable
     */
    public void setRampingEnabled(boolean enabled) {
        rampingEnabled = enabled;
        if (!enabled) {
            currentPower = targetPower;
            applyPower();
        }
    }
    
    /**
     * Check if power ramping is enabled.
     * 
     * @return true if ramping is enabled
     */
    public boolean isRampingEnabled() {
        return rampingEnabled;
    }
    
    /**
     * Set the ramp rate for power transitions.
     * 
     * @param rampRate Power change per update (typically 0.01 to 0.1)
     */
    public void setRampRate(double rampRate) {
        this.rampRate = Math.abs(rampRate);
    }
    
    /**
     * Get the current ramp rate.
     * 
     * @return Current ramp rate
     */
    public double getRampRate() {
        return rampRate;
    }
    
    /**
     * Emergency stop - immediately stops intake and sets error state.
     */
    public void emergencyStop() {
        if (intakeServo != null) {
            intakeServo.setPower(0.0);
        }
        currentPower = 0.0;
        targetPower = 0.0;
        running = false;
        state = State.ERROR;
        rampingEnabled = false;
        stopSafetyTimer();
        
        if (telemetry != null && telemetryEnabled) {
            telemetry.addLine("EMERGENCY STOP: Intake stopped");
            telemetry.update();
        }
    }
    
    /**
     * Enable or disable safety timeout.
     * When enabled, intake will automatically stop after MAX_RUNTIME_MS.
     * 
     * @param enabled true to enable safety timeout
     */
    public void setSafetyTimeoutEnabled(boolean enabled) {
        safetyTimeoutEnabled = enabled;
        if (!enabled) {
            stopSafetyTimer();
        }
    }
    
    /**
     * Check if safety timeout is enabled.
     * 
     * @return true if safety timeout is enabled
     */
    public boolean isSafetyTimeoutEnabled() {
        return safetyTimeoutEnabled;
    }
    
    /**
     * Update intake state machine and handle power ramping.
     * Should be called every loop iteration.
     */
    public void update() {
        if (!initialized || intakeServo == null) {
            return;
        }
        
        // Check safety timeout
        if (safetyTimeoutEnabled && running && startTime > 0) {
            long runtime = System.currentTimeMillis() - startTime;
            if (runtime > IntakeConstants.MAX_RUNTIME_MS) {
                emergencyStop();
                if (telemetry != null && telemetryEnabled) {
                    telemetry.addLine("Intake stopped: Safety timeout exceeded");
                }
                return;
            }
        }
        
        // Handle power ramping
        if (rampingEnabled) {
            double powerDiff = targetPower - currentPower;
            if (Math.abs(powerDiff) > rampRate) {
                // Ramp towards target
                currentPower += Math.signum(powerDiff) * rampRate;
            } else {
                // Reached target
                currentPower = targetPower;
            }
            applyPower();
        }
        
        // Update running state
        running = Math.abs(currentPower) > IntakeConstants.POWER_THRESHOLD;
        
        // Update state based on power
        if (!running && state != State.ERROR) {
            state = State.IDLE;
            stopSafetyTimer();
        }
        
        // Update telemetry
        if (telemetryEnabled && telemetry != null) {
            telemetry.addData("Intake Power", "%.2f", currentPower);
            telemetry.addData("Intake Target", "%.2f", targetPower);
            telemetry.addData("Intake Status", state.toString());
            telemetry.addData("Intake Running", running ? "Yes" : "No");
            if (rampingEnabled) {
                telemetry.addData("Intake Ramping", "Yes (%.3f/update)", rampRate);
            }
        }
    }
    
    /**
     * Apply current power to hardware.
     * Private helper method.
     */
    private void applyPower() {
        if (intakeServo != null && initialized) {
            intakeServo.setPower(currentPower);
        }
    }
    
    /**
     * Start safety timer.
     * Private helper method.
     */
    private void startSafetyTimer() {
        if (safetyTimeoutEnabled) {
            startTime = System.currentTimeMillis();
        }
    }
    
    /**
     * Stop safety timer.
     * Private helper method.
     */
    private void stopSafetyTimer() {
        startTime = 0;
    }
}

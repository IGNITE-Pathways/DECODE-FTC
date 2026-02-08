package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TuningController - Automated test sequence for PIDF tuning
 * Cycles through different speed targets to test controller performance
 */
public class TuningController {
    // Motor specifications
    public static final double MOTOR_TICKS_PER_REV = 28;
    public static final double MOTOR_MAX_RPM = 5400;
    public static final double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    // Speed range for testing
    public static final double TESTING_MAX_SPEED = 0.9 * MOTOR_MAX_RPM; // 4860 RPM
    public static final double TESTING_MIN_SPEED = 0.3 * MOTOR_MAX_RPM; // 1620 RPM

    // State durations (in seconds)
    public static final double STATE1_RAMPING_UP_DURATION = 3.5;
    public static final double STATE2_COASTING_1_DURATION = 4.0;
    public static final double STATE3_RAMPING_DOWN_DURATION = 2.0;
    public static final double STATE4_COASTING_2_DURATION = 2.0;
    public static final double STATE5_RANDOM_1_DURATION = 2.0;
    public static final double STATE6_RANDOM_2_DURATION = 2.0;
    public static final double STATE7_RANDOM_3_DURATION = 2.0;
    public static final double STATE8_REST_DURATION = 1.0;

    // State enumeration
    private enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    // State machine variables
    private State currentState = State.RAMPING_UP;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime progressTimer = new ElapsedTime();

    private double currentTargetVelo = 0.0;
    private double random1Target = 0.0;
    private double random2Target = 0.0;
    private double random3Target = 0.0;

    public TuningController() {
        // Generate random targets for random states
        random1Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
        random2Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
        random3Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
    }

    /**
     * Start or restart the tuning sequence
     */
    public void start() {
        currentState = State.RAMPING_UP;
        stateTimer.reset();
        progressTimer.reset();

        // Regenerate random targets
        random1Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
        random2Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
        random3Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
    }

    /**
     * Update the state machine and calculate target velocity
     * Call this every loop iteration
     * @return Target velocity in ticks per second
     */
    public double update() {
        double stateTime = stateTimer.seconds();

        // Check if we need to transition to next state
        boolean shouldTransition = false;

        switch (currentState) {
            case RAMPING_UP:
                if (stateTime >= STATE1_RAMPING_UP_DURATION) {
                    shouldTransition = true;
                } else {
                    // Calculate ramping velocity
                    double progress = progressTimer.seconds() / STATE1_RAMPING_UP_DURATION;
                    double targetRPM = progress * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
                    currentTargetVelo = rpmToTicksPerSecond(targetRPM);
                }
                break;

            case COASTING_1:
                if (stateTime >= STATE2_COASTING_1_DURATION) {
                    shouldTransition = true;
                } else {
                    currentTargetVelo = rpmToTicksPerSecond(TESTING_MAX_SPEED);
                }
                break;

            case RAMPING_DOWN:
                if (stateTime >= STATE3_RAMPING_DOWN_DURATION) {
                    shouldTransition = true;
                } else {
                    // Calculate ramping velocity
                    double progress = progressTimer.seconds() / STATE3_RAMPING_DOWN_DURATION;
                    double targetRPM = TESTING_MAX_SPEED - progress * (TESTING_MAX_SPEED - TESTING_MIN_SPEED);
                    currentTargetVelo = rpmToTicksPerSecond(targetRPM);
                }
                break;

            case COASTING_2:
                if (stateTime >= STATE4_COASTING_2_DURATION) {
                    shouldTransition = true;
                } else {
                    currentTargetVelo = rpmToTicksPerSecond(TESTING_MIN_SPEED);
                }
                break;

            case RANDOM_1:
                if (stateTime >= STATE5_RANDOM_1_DURATION) {
                    shouldTransition = true;
                } else {
                    currentTargetVelo = rpmToTicksPerSecond(random1Target);
                }
                break;

            case RANDOM_2:
                if (stateTime >= STATE6_RANDOM_2_DURATION) {
                    shouldTransition = true;
                } else {
                    currentTargetVelo = rpmToTicksPerSecond(random2Target);
                }
                break;

            case RANDOM_3:
                if (stateTime >= STATE7_RANDOM_3_DURATION) {
                    shouldTransition = true;
                } else {
                    currentTargetVelo = rpmToTicksPerSecond(random3Target);
                }
                break;

            case REST:
                if (stateTime >= STATE8_REST_DURATION) {
                    shouldTransition = true;
                } else {
                    currentTargetVelo = 0;
                }
                break;
        }

        // Transition to next state if needed
        if (shouldTransition) {
            transitionToNextState();
        }

        return currentTargetVelo;
    }

    /**
     * Transition to the next state in the sequence
     */
    private void transitionToNextState() {
        // Determine next state
        switch (currentState) {
            case RAMPING_UP:
                currentState = State.COASTING_1;
                break;
            case COASTING_1:
                currentState = State.RAMPING_DOWN;
                break;
            case RAMPING_DOWN:
                currentState = State.COASTING_2;
                break;
            case COASTING_2:
                currentState = State.RANDOM_1;
                break;
            case RANDOM_1:
                currentState = State.RANDOM_2;
                break;
            case RANDOM_2:
                currentState = State.RANDOM_3;
                break;
            case RANDOM_3:
                currentState = State.REST;
                break;
            case REST:
                // Loop back to beginning
                currentState = State.RAMPING_UP;
                // Generate new random targets for next cycle
                random1Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
                random2Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
                random3Target = Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;
                break;
        }

        // Reset timers for new state
        stateTimer.reset();
        progressTimer.reset();
    }

    /**
     * Get the current state name (for telemetry)
     */
    public String getCurrentStateName() {
        return currentState.toString();
    }

    /**
     * Get the time elapsed in the current state
     */
    public double getCurrentStateTime() {
        return stateTimer.seconds();
    }

    /**
     * Convert RPM to ticks per second
     */
    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60.0;
    }

    /**
     * Convert ticks per second to RPM
     */
    public static double ticksPerSecondToRPM(double ticksPerSec) {
        return (ticksPerSec / MOTOR_TICKS_PER_REV) * MOTOR_GEAR_RATIO * 60.0;
    }
}

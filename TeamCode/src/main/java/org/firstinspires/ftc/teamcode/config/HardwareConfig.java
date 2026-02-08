package org.firstinspires.ftc.teamcode.config;

/**
 * Hardware configuration - all device names in one place.
 *
 * ACTIVE HARDWARE:
 * - 4 drive motors (mecanum)
 * - 2 flywheel motors
 * - Hood servo
 * - Turret servo
 * - Transfer servo
 * - Intake motor
 * - Limelight camera
 * - IMU (in Pinpoint)
 */
public class HardwareConfig {

    private HardwareConfig() {
        throw new IllegalStateException("Utility class");
    }

    // ========== DRIVE MOTORS ==========
    public static final String LEFT_FRONT_MOTOR = "leftfrontmotor";
    public static final String RIGHT_FRONT_MOTOR = "rightfrontmotor";
    public static final String LEFT_BACK_MOTOR = "leftbackmotor";
    public static final String RIGHT_BACK_MOTOR = "rightbackmotor";

    // ========== SHOOTER ==========
    public static final String FLYWHEEL_MOTOR = "flywheelmotor";
    public static final String FLYWHEEL_MOTOR_2 = "flywheelmotor2";
    public static final String HOOD_SERVO = "hoodservo";

    // ========== TURRET ==========
    public static final String TURRET_SERVO = "turretServo";

    // ========== INTAKE ==========
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String TRANSFER_SERVO = "transferServo";

    // ========== SENSORS ==========
    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu";
}

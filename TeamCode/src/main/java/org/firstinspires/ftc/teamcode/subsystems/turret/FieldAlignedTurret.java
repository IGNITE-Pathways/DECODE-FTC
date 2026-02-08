package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.TurretConstants;

/**
 * Field-Aligned Turret Component
 *
 * Automatically aligns the turret to a fixed goal position on the field using Pinpoint odometry.
 * Can be used in both TeleOp and Autonomous modes.
 *
 * Features:
 * - Uses Pinpoint odometry for robot position tracking
 * - Calculates turret angle to always point at the goal
 * - Supports both BLUE and RED alliance goals
 * - Can track custom goal positions
 *
 * Usage:
 * 1. Call initialize() with hardwareMap, telemetry, and alliance
 * 2. Call setEnabled(true) to start tracking
 * 3. Call update() in your loop to continuously update turret position
 * 4. Call setEnabled(false) to stop tracking
 */
public class FieldAlignedTurret {

    // ======= GOAL COORDINATES (Pedro inches) =======
    private static final double BLUE_GOAL_X_IN = 12.0;
    private static final double RED_GOAL_X_IN = 132.0;
    private static final double GOAL_Y_IN = 137.0;

    // Pinpoint heading convention: 0°=+Y (forward). Pedro uses: 90°=+Y (forward).
    private static final double PINPOINT_TO_PEDRO_DEG = 90.0;

    // Hardware
    private Servo turretServo;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;

    // State
    private AllianceColor alliance;
    private boolean enabled = false;
    private boolean initialized = false;
    private double headingOffsetDeg = 0.0;

    // Custom goal tracking
    private boolean useCustomGoal = false;
    private double customGoalX = 0.0;
    private double customGoalY = 0.0;

    // Latest calculation results
    private TurretCalculation lastCalculation;

    /**
     * Initialize the field-aligned turret component
     *
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry for debugging
     * @param alliance Alliance color (BLUE or RED) to determine goal position
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance) {
        this.telemetry = telemetry;
        this.alliance = alliance;

        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            turretServo.setPosition(TurretConstants.SERVO_CENTER);
        } catch (Exception e) {
            telemetry.addLine("✗ Turret servo NOT FOUND");
            turretServo = null;
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
            configurePinpoint();
        } catch (Exception e) {
            telemetry.addLine("✗ Pinpoint odometry NOT FOUND");
            pinpoint = null;
        }

        initialized = (turretServo != null && pinpoint != null);

        if (initialized) {
            telemetry.addLine("✓ Field-Aligned Turret initialized");
        } else {
            telemetry.addLine("✗ Field-Aligned Turret initialization FAILED");
        }
    }

    /**
     * Configure Pinpoint odometry with robot parameters
     */
    private void configurePinpoint() {
        pinpoint.setOffsets(
            RobotConstants.PINPOINT_STRAFE_POD_X,
            RobotConstants.PINPOINT_FORWARD_POD_Y,
            DistanceUnit.INCH
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        headingOffsetDeg = 0.0;
    }

    /**
     * Set the initial pose for odometry (useful for autonomous)
     *
     * @param xInches X position in inches (Pedro coordinates)
     * @param yInches Y position in inches (Pedro coordinates)
     * @param headingDegrees Heading in degrees (Pedro convention: 90° = forward)
     */
    public void setStartPose(double xInches, double yInches, double headingDegrees) {
        if (pinpoint == null) return;

        // Convert Pedro heading to Pinpoint heading convention
        double startPinpointHeadingDeg = headingDegrees - PINPOINT_TO_PEDRO_DEG;
        pinpoint.setPosition(new Pose2D(
            DistanceUnit.INCH,
            xInches,
            yInches,
            AngleUnit.DEGREES,
            startPinpointHeadingDeg
        ));
        headingOffsetDeg = 0.0;
    }

    /**
     * Enable or disable turret tracking
     *
     * @param enabled true to enable tracking, false to disable
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;

        // Reset to center when disabled
        if (!enabled && turretServo != null) {
            turretServo.setPosition(TurretConstants.SERVO_CENTER);
        }
    }

    /**
     * Check if turret tracking is enabled
     *
     * @return true if tracking is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Set a custom goal position (instead of using alliance default)
     *
     * @param xInches Goal X coordinate in inches
     * @param yInches Goal Y coordinate in inches
     */
    public void setCustomGoal(double xInches, double yInches) {
        this.customGoalX = xInches;
        this.customGoalY = yInches;
        this.useCustomGoal = true;
    }

    /**
     * Use the default alliance goal position
     */
    public void useAllianceGoal() {
        this.useCustomGoal = false;
    }

    /**
     * Update the turret position based on current robot pose
     * Call this method in your loop to continuously track the goal
     *
     * @return true if turret was updated successfully
     */
    public boolean update() {
        if (!initialized || !enabled || pinpoint == null || turretServo == null) {
            return false;
        }

        // Update odometry
        pinpoint.update();

        // Get current robot pose
        Pose2D pose = pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES) - headingOffsetDeg;
        double robotHeadingRad = Math.toRadians(robotHeadingDeg + PINPOINT_TO_PEDRO_DEG);

        // Determine goal position
        double goalX, goalY;
        if (useCustomGoal) {
            goalX = customGoalX;
            goalY = customGoalY;
        } else {
            goalX = (alliance == AllianceColor.BLUE) ? BLUE_GOAL_X_IN : RED_GOAL_X_IN;
            goalY = GOAL_Y_IN;
        }

        // Calculate turret position
        lastCalculation = computeTurretToGoal(
            robotX,
            robotY,
            robotHeadingRad,
            goalX,
            goalY
        );

        // Apply turret position
        turretServo.setPosition(lastCalculation.turretServoPosition);

        return true;
    }

    /**
     * Get the current servo position
     *
     * @return Servo position (0.0 to 1.0)
     */
    public double getServoPosition() {
        if (turretServo == null) return TurretConstants.SERVO_CENTER;
        return turretServo.getPosition();
    }

    /**
     * Manually set the turret servo position (disables auto-tracking)
     *
     * @param position Servo position (0.0 to 1.0)
     */
    public void setPositionDirect(double position) {
        if (turretServo == null) return;
        enabled = false;  // Disable auto-tracking
        turretServo.setPosition(clamp(position, 0.0, 1.0));
    }

    /**
     * Get the last calculation result (for telemetry/debugging)
     *
     * @return Last turret calculation result, or null if never calculated
     */
    public TurretCalculation getLastCalculation() {
        return lastCalculation;
    }

    /**
     * Get the current robot pose from Pinpoint odometry
     *
     * @return Current robot pose (Pedro coordinates), or null if Pinpoint unavailable
     */
    public RobotPose getCurrentPose() {
        if (pinpoint == null) return null;

        Pose2D pose = pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES) - headingOffsetDeg + PINPOINT_TO_PEDRO_DEG;

        return new RobotPose(robotX, robotY, robotHeadingDeg);
    }

    /**
     * Get the current goal position being tracked
     *
     * @return Goal position in inches {x, y}
     */
    public double[] getGoalPosition() {
        if (useCustomGoal) {
            return new double[]{customGoalX, customGoalY};
        } else {
            double goalX = (alliance == AllianceColor.BLUE) ? BLUE_GOAL_X_IN : RED_GOAL_X_IN;
            return new double[]{goalX, GOAL_Y_IN};
        }
    }

    /**
     * Check if turret is within tolerance of desired angle
     *
     * @param toleranceDeg Tolerance in degrees
     * @return true if turret is aligned within tolerance
     */
    public boolean isAligned(double toleranceDeg) {
        if (lastCalculation == null) return false;

        // Check if desired angle is achievable (within turret range)
        if (lastCalculation.turretAngleDegDesired < 0.0 || lastCalculation.turretAngleDegDesired > 180.0) {
            return false;  // Out of range
        }

        // Check if we're within tolerance
        double error = Math.abs(lastCalculation.turretAngleDegDesired - lastCalculation.turretAngleDegCmd);
        return error <= toleranceDeg;
    }

    /**
     * Stop the turret and reset to center
     */
    public void stop() {
        setEnabled(false);
    }

    /**
     * Compute the turret servo position needed to point at a goal
     *
     * @param robotXIn Robot X position in inches
     * @param robotYIn Robot Y position in inches
     * @param robotHeadingRad Robot heading in radians (Pedro convention)
     * @param goalXIn Goal X position in inches
     * @param goalYIn Goal Y position in inches
     * @return Turret calculation result
     */
    private static TurretCalculation computeTurretToGoal(
            double robotXIn,
            double robotYIn,
            double robotHeadingRad,
            double goalXIn,
            double goalYIn
    ) {
        // Calculate angle from robot to goal
        double angleToGoalRad = Math.atan2(goalYIn - robotYIn, goalXIn - robotXIn);

        // Calculate relative angle (goal angle relative to robot heading)
        double relativeRad = wrapRadians(angleToGoalRad - robotHeadingRad);
        double relativeDeg = Math.toDegrees(relativeRad);

        // Convert to turret angle (0°=left, 90°=center/forward, 180°=right)
        double turretDegDesired = 90.0 - relativeDeg;

        // Clamp to turret physical limits
        double turretDegCmd = clamp(turretDegDesired, 0.0, 180.0);

        // Convert to servo position (0°->0.0, 90°->0.5, 180°->1.0)
        double servoPos = clamp(turretDegCmd / 180.0, 0.0, 1.0);

        return new TurretCalculation(
            servoPos,
            turretDegDesired,
            turretDegCmd,
            angleToGoalRad,
            relativeDeg
        );
    }

    /**
     * Wrap radians to [-π, π]
     */
    private static double wrapRadians(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }

    /**
     * Clamp a value between min and max
     */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Result of turret calculation
     */
    public static class TurretCalculation {
        public final double turretServoPosition;      // Servo position (0.0 to 1.0)
        public final double turretAngleDegDesired;    // Desired turret angle (may be out of range)
        public final double turretAngleDegCmd;        // Commanded turret angle (clamped to range)
        public final double angleToGoalRad;           // Absolute angle to goal in radians
        public final double relativeToRobotDeg;       // Angle relative to robot heading in degrees

        public TurretCalculation(
                double turretServoPosition,
                double turretAngleDegDesired,
                double turretAngleDegCmd,
                double angleToGoalRad,
                double relativeToRobotDeg
        ) {
            this.turretServoPosition = turretServoPosition;
            this.turretAngleDegDesired = turretAngleDegDesired;
            this.turretAngleDegCmd = turretAngleDegCmd;
            this.angleToGoalRad = angleToGoalRad;
            this.relativeToRobotDeg = relativeToRobotDeg;
        }
    }

    /**
     * Robot pose in Pedro coordinates
     */
    public static class RobotPose {
        public final double xInches;
        public final double yInches;
        public final double headingDegrees;

        public RobotPose(double xInches, double yInches, double headingDegrees) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingDegrees = headingDegrees;
        }
    }
}

package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.TurretConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Test OpMode: continuously auto-align turret to the fixed GOAL coordinate.
 *
 * Uses Pinpoint odometry coordinates + IMU heading (Pose2D) and computes the turret servo position via trig:
 *   angleToGoal = atan2(goalY - robotY, goalX - robotX)
 *   relative    = angleToGoal - robotHeading
 *   turretDeg   = 90 - relativeDeg   (0=left, 90=center/forward, 180=right)
 *
 * IMPORTANT:
 * - Update START_X_IN / START_Y_IN / START_HEADING_DEG to your initial pose for testing.
 * - Goal coordinates are (BLUE: 12,137) and (RED: 132,137) in Pedro/field inches.
 */
@TeleOp(name = "Test: Turret Field Auto Align", group = "Test")
public class TurretFieldAutoAlignTest extends LinearOpMode {

    // ======= YOUR INITIAL POSE (edit these variables) =======
    private static final double START_X_IN = 56.0;
    private static final double START_Y_IN = 38.0;
    private static final double START_HEADING_DEG = 90.0; // Pedro forward

    // Pinpoint heading behaves like: 0°=+Y (forward). Pedro uses: 90°=+Y (forward).
    private static final double PINPOINT_TO_PEDRO_DEG = 90.0;

    // ======= SELECT ALLIANCE TARGET (edit this variable) =======
    private static final AllianceColor ALLIANCE = AllianceColor.BLUE;

    // ======= GOAL COORDINATES (Pedro inches) =======
    private static final double BLUE_GOAL_X_IN = 12.0;
    private static final double RED_GOAL_X_IN = 132.0;
    private static final double GOAL_Y_IN = 137.0;

    // ======= Pinpoint configuration (must match robot) =======
    private static final double FORWARD_POD_Y = 3.75;
    private static final double STRAFE_POD_X = -7.08661;

    private GoBildaPinpointDriver pinpoint;
    private Servo turretServo;

    private double headingOffsetDeg = 0.0;

    @Override
    public void runOpMode() {
        turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
        turretServo.setPosition(TurretConstants.SERVO_CENTER);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
        configurePinpoint();

        // Seed odometry pose from your Pedro coordinates
        setStartPose();

        telemetry.addLine("=== Turret Field Auto Align Test ===");
        telemetry.addLine("Turret continuously tracks the goal while enabled.");
        telemetry.addData("START", "(%.1f, %.1f, %.1f°)", START_X_IN, START_Y_IN, START_HEADING_DEG);
        telemetry.addData("ALLIANCE", ALLIANCE);
        telemetry.addData("Goal BLUE", "(%.0f, %.0f)", BLUE_GOAL_X_IN, GOAL_Y_IN);
        telemetry.addData("Goal RED", "(%.0f, %.0f)", RED_GOAL_X_IN, GOAL_Y_IN);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES) - headingOffsetDeg;
            double robotHeadingRad = Math.toRadians(robotHeadingDeg + PINPOINT_TO_PEDRO_DEG);

            double goalX = (ALLIANCE == AllianceColor.BLUE) ? BLUE_GOAL_X_IN : RED_GOAL_X_IN;
            double goalY = GOAL_Y_IN;

            TurretResult r = computeTurretToGoal(
                    robotX,
                    robotY,
                    robotHeadingRad,
                    goalX,
                    goalY
            );

            turretServo.setPosition(r.turretServoPosition);

            telemetry.addLine("=== Turret Field Auto Align Test ===");
            telemetry.addData("Alliance", ALLIANCE);
            telemetry.addData("Goal", "(%.1f, %.1f)", goalX, goalY);
            telemetry.addLine();

            telemetry.addLine("--- Pose (Pinpoint) ---");
            telemetry.addData("X", "%.2f in", robotX);
            telemetry.addData("Y", "%.2f in", robotY);
            telemetry.addData("Heading", "%.2f°", robotHeadingDeg);
            telemetry.addData("HeadingOffset", "%.2f°", headingOffsetDeg);
            telemetry.addLine();

            telemetry.addLine("--- Turret Setpoint ---");
            telemetry.addData("AngleToGoal", "%.2f°", Math.toDegrees(r.angleToGoalRad));
            telemetry.addData("Relative (L+/R-)", "%.2f°", r.relativeToRobotDeg);
            telemetry.addData("TurretDeg desired", "%.2f°", r.turretAngleDegDesired);
            telemetry.addData("TurretDeg cmd", "%.2f°", r.turretAngleDegCmd);
            telemetry.addData("Servo(0..1)", "%.4f", r.turretServoPosition);
            if (r.turretAngleDegDesired < 0.0 || r.turretAngleDegDesired > 180.0) {
                telemetry.addLine("NOTE: Desired turret angle outside range; output clamped to endstop.");
            }
            telemetry.update();

            idle();
        }
    }

    private void configurePinpoint() {
        // Same values used elsewhere in this codebase
        pinpoint.setOffsets(STRAFE_POD_X, FORWARD_POD_Y, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        headingOffsetDeg = 0.0;
    }

    private void setStartPose() {
        // Seed pose: convert desired Pedro heading into Pinpoint's heading convention.
        double startPinpointHeadingDeg = START_HEADING_DEG - PINPOINT_TO_PEDRO_DEG;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN, AngleUnit.DEGREES, startPinpointHeadingDeg));
        headingOffsetDeg = 0.0;
    }

    private static TurretResult computeTurretToGoal(
            double robotXIn,
            double robotYIn,
            double robotHeadingRad,
            double goalXIn,
            double goalYIn
    ) {
        double angleToGoalRad = Math.atan2(goalYIn - robotYIn, goalXIn - robotXIn);
        double relativeRad = wrapRadians(angleToGoalRad - robotHeadingRad);
        double relativeDeg = Math.toDegrees(relativeRad);

        double turretDegDesired = 90.0 - relativeDeg;
        double turretDegCmd = clamp(turretDegDesired, 0.0, 180.0);

        // Requested mapping: 0°->0.0, 90°->0.5, 180°->1.0
        double servoPos = clamp(turretDegCmd / 180.0, 0.0, 1.0);

        return new TurretResult(servoPos, turretDegDesired, turretDegCmd, angleToGoalRad, relativeDeg);
    }

    private static double wrapRadians(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static final class TurretResult {
        final double turretServoPosition;
        final double turretAngleDegDesired;
        final double turretAngleDegCmd;
        final double angleToGoalRad;
        final double relativeToRobotDeg;

        TurretResult(double turretServoPosition, double turretAngleDegDesired, double turretAngleDegCmd, double angleToGoalRad, double relativeToRobotDeg) {
            this.turretServoPosition = turretServoPosition;
            this.turretAngleDegDesired = turretAngleDegDesired;
            this.turretAngleDegCmd = turretAngleDegCmd;
            this.angleToGoalRad = angleToGoalRad;
            this.relativeToRobotDeg = relativeToRobotDeg;
        }
    }
}


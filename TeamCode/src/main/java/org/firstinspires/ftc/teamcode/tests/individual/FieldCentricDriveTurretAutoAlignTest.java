package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;

/**
 * Drive test: CompetitionTeleOp-style ROBOT-CENTRIC driving + turret automatically points at the GOAL every loop.
 *
 * Drive controls (matches CompetitionTeleOpBase):
 * - Left stick: fwd/strafe (robot-centric)
 * - Right stick X: rotate
 *
 * Turret:
 * - No turret buttons. It always updates live from (x,y,heading) -> goal.
 */
@TeleOp(name = "Test: TeleOpDrive + Turret AutoAlign", group = "Test")
public class FieldCentricDriveTurretAutoAlignTest extends LinearOpMode {

    // ======= SELECT ALLIANCE TARGET (edit this variable) =======
    private static final AllianceColor ALLIANCE = AllianceColor.BLUE;

    // ======= STARTING FIELD POSE (Pedro inches/degrees) =======
    // IMPORTANT: If you don't set this to your real field start, the turret will aim as if you started at (0,0).
    private static final double START_X_IN = 56.0;
    private static final double START_Y_IN = 38.0;
    private static final double START_HEADING_DEG = 90.0;

    // ======= HEADING CONVENTION BRIDGE =======
    // Pinpoint heading (as used by FieldCentricDrive math) behaves like: 0° = +Y (forward), 90° = +X (right).
    // Pedro/trig uses: 0° = +X (right), 90° = +Y (forward).
    // So: pedroHeadingDeg = pinpointHeadingDeg + 90
    private static final double PINPOINT_TO_PEDRO_DEG = 90.0;

    // ======= GOAL COORDINATES (Pedro inches) =======
    private static final double BLUE_GOAL_X_IN = 12.0;
    private static final double RED_GOAL_X_IN = 132.0;
    private static final double GOAL_Y_IN = 137.0;

    // ======= Pinpoint configuration (must match robot) =======
    private static final double FORWARD_POD_Y = 3.75;
    private static final double STRAFE_POD_X = -7.08661;

    private DriveTrain driveTrain;

    private Servo turretServo;
    private GoBildaPinpointDriver pinpoint;

    // Offsets that map Pinpoint's arbitrary origin into your Pedro field pose.
    // These get computed once after start, after Pinpoint settles.
    private double fieldXOffsetIn = 0.0;
    private double fieldYOffsetIn = 0.0;
    private double pinpointHeadingOffsetDeg = 0.0; // applied to raw Pinpoint heading
    private boolean offsetsInitialized = false;

    // Debug: detect heading freezes / jumps
    private double lastPinpointHeadingDeg = Double.NaN;
    private double lastTurretServoPos = Double.NaN;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);
        driveTrain.setInputCurve(RobotConstants.DRIVE_INPUT_CURVE);
        driveTrain.setRotationSensitivity(RobotConstants.ROTATION_SENSITIVITY);
        driveTrain.setBatteryCompensationEnabled(RobotConstants.ENABLE_BATTERY_COMPENSATION);

        // Turret servo
        turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
        turretServo.setPosition(TurretConstants.SERVO_CENTER);

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
        configurePinpoint();

        telemetry.addLine("=== TeleOpDrive + Turret AutoAlign ===");
        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("StartPose", "(%.1f, %.1f, %.1f°)", START_X_IN, START_Y_IN, START_HEADING_DEG);
        telemetry.addLine("Drive: LS=fwd/strafe, RSx=rotate (robot-centric)");
        telemetry.addLine("Turret: automatic (no buttons)");
        telemetry.addLine("NOTE: Init holds turret; offsets lock before START.");
        telemetry.update();

        // INIT LOOP:
        // - Keep turret held (so your init alignment stays perfect)
        // - Initialize offsets once Pinpoint reports READY (so it won't jump on START)
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            if (!offsetsInitialized && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                initializeFieldOffsetsFromCurrentPose();
            }

            // Hold turret on init (do NOT auto-aim yet)
            turretServo.setPosition(TurretConstants.SERVO_CENTER);

            telemetry.addLine("=== TeleOpDrive + Turret AutoAlign (INIT) ===");
            telemetry.addData("PinpointStatus", pinpoint.getDeviceStatus());
            telemetry.addData("OffsetsInit", offsetsInitialized ? "YES" : "NO");
            if (offsetsInitialized) {
                // Show the pose that WILL be used on start
                double rawPinpointHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
                double robotX = pose.getX(DistanceUnit.INCH) + fieldXOffsetIn;
                double robotY = pose.getY(DistanceUnit.INCH) + fieldYOffsetIn;
                double pinpointHeadingDeg = rawPinpointHeadingDeg + pinpointHeadingOffsetDeg;
                telemetry.addData("PoseUsed(onStart)", "X=%.1f Y=%.1f H(pedro)=%.1f°", robotX, robotY, pinpointHeadingDeg + PINPOINT_TO_PEDRO_DEG);
            } else {
                telemetry.addLine("Waiting for Pinpoint READY to lock start pose offsets...");
            }
            telemetry.update();

            idle();
        }

        // If we never saw READY during init, fall back to initializing offsets at start.
        if (!offsetsInitialized) {
            for (int i = 0; i < 8 && opModeIsActive(); i++) {
                pinpoint.update();
                idle();
            }
            initializeFieldOffsetsFromCurrentPose();
        }

        while (opModeIsActive()) {
            pinpoint.update();

            // ===== Drive inputs (robot-centric, same shaping as CompetitionTeleOpBase) =====
            double fwd = -gamepad1.left_stick_y;
            double str = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            // Apply joystick deadzone
            fwd = applyDeadzone(fwd);
            str = applyDeadzone(str);
            rot = applyDeadzone(rot);

            // Apply input curve to translation vector magnitude (preserves direction)
            double magnitude = Math.sqrt(fwd * fwd + str * str);
            if (magnitude > 0.01) {
                double curvedMagnitude = Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
                double scale = curvedMagnitude / magnitude;
                fwd *= scale;
                str *= scale;
            }

            // Apply input curve to rotation separately
            rot = applyInputCurve(rot);
            // Apply rotation sensitivity
            rot *= RobotConstants.ROTATION_SENSITIVITY;

            driveTrain.driveRaw(fwd, str, rot);

            // ===== Pose for turret =====
            Pose2D pose = pinpoint.getPosition();
            updateTurretAndTelemetry(pose, fwd, str, rot, false);

            idle();
        }
    }

    private void updateTurretAndTelemetry(Pose2D pose, double fwd, double str, double rot, boolean isInit) {
        double rawPinpointHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

        // Apply our offsets so the turret math uses a consistent field coordinate frame.
        double robotX = pose.getX(DistanceUnit.INCH) + fieldXOffsetIn;
        double robotY = pose.getY(DistanceUnit.INCH) + fieldYOffsetIn;
        double pinpointHeadingDeg = rawPinpointHeadingDeg + pinpointHeadingOffsetDeg;
        double pedroHeadingRadForTurret = Math.toRadians(pinpointHeadingDeg + PINPOINT_TO_PEDRO_DEG);

        double goalX = (ALLIANCE == AllianceColor.BLUE) ? BLUE_GOAL_X_IN : RED_GOAL_X_IN;
        double goalY = GOAL_Y_IN;

        // Compute once so telemetry matches the command exactly
        double angleToGoalRad = Math.atan2(goalY - robotY, goalX - robotX);
        double relativeRad = wrapRadians(angleToGoalRad - pedroHeadingRadForTurret);
        double relativeDeg = Math.toDegrees(relativeRad);

        double turretDegDesired = 90.0 - relativeDeg;
        double turretDegCmd = clamp(turretDegDesired, 0.0, 180.0);
        double turretServoPos = clamp(turretDegCmd / 180.0, 0.0, 1.0);

        turretServo.setPosition(turretServoPos);

        telemetry.addLine(isInit ? "=== TeleOpDrive + Turret AutoAlign (INIT) ===" : "=== TeleOpDrive + Turret AutoAlign ===");
        telemetry.addData("PinpointStatus", pinpoint.getDeviceStatus());
        telemetry.addData("OffsetsInit", offsetsInitialized ? "YES" : "NO");
        telemetry.addData("PoseUsed", "X=%.1f Y=%.1f H(pedro)=%.1f°", robotX, robotY, pinpointHeadingDeg + PINPOINT_TO_PEDRO_DEG);
        telemetry.addData("X", "%.1f", robotX);
        telemetry.addData("Y", "%.1f", robotY);
        telemetry.addData("Heading (Pinpoint)", "%.1f°", pinpointHeadingDeg);
        telemetry.addData("Heading (Pedro)", "%.1f°", pinpointHeadingDeg + PINPOINT_TO_PEDRO_DEG);
        telemetry.addData("dx", "%.1f", goalX - robotX);
        telemetry.addData("dy", "%.1f", goalY - robotY);
        telemetry.addLine();
        telemetry.addLine("--- Turret Math ---");
        telemetry.addData("AngleToGoal", "%.1f°", Math.toDegrees(angleToGoalRad));
        telemetry.addData("Relative", "%.1f°", relativeDeg);
        telemetry.addData("TurretDeg (desired)", "%.1f°", turretDegDesired);
        telemetry.addData("TurretDeg (cmd)", "%.1f°", turretDegCmd);
        telemetry.addData("TurretServo(0..1)", "%.4f", turretServoPos);
        if (turretDegDesired < 0.0 || turretDegDesired > 180.0) {
            telemetry.addLine("NOTE: Desired turret angle outside range; output clamped to endstop.");
        }

        if (!Double.isNaN(lastPinpointHeadingDeg)) {
            double headingDelta = Math.abs(pinpointHeadingDeg - lastPinpointHeadingDeg);
            double servoDelta = Double.isNaN(lastTurretServoPos) ? 0.0 : Math.abs(turretServoPos - lastTurretServoPos);

            if (Math.abs(rot) > 0.25 && headingDelta < 0.2) {
                telemetry.addLine("WARNING: Rotating input but heading not changing (check Pinpoint/IMU).");
            }
            if (headingDelta > 1.0 && servoDelta < 0.001 && (turretDegCmd < 1.0 || turretDegCmd > 179.0)) {
                telemetry.addLine("NOTE: Turret setpoint clamped at endstop (0° or 180°).");
            }
        }
        lastPinpointHeadingDeg = pinpointHeadingDeg;
        lastTurretServoPos = turretServoPos;

        telemetry.update();
    }

    private void initializeFieldOffsetsFromCurrentPose() {
        // Compute offsets so that the current Pinpoint pose maps to the desired Pedro field pose.
        Pose2D pose = pinpoint.getPosition();

        double rawX = pose.getX(DistanceUnit.INCH);
        double rawY = pose.getY(DistanceUnit.INCH);
        double rawHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

        // Convert desired Pedro heading into the equivalent Pinpoint heading.
        double desiredPinpointHeadingDeg = START_HEADING_DEG - PINPOINT_TO_PEDRO_DEG;

        fieldXOffsetIn = START_X_IN - rawX;
        fieldYOffsetIn = START_Y_IN - rawY;
        pinpointHeadingOffsetDeg = desiredPinpointHeadingDeg - rawHeadingDeg;
        offsetsInitialized = true;
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < RobotConstants.JOYSTICK_DEAD_ZONE) {
            return 0;
        }
        double sign = Math.signum(input);
        return sign * (Math.abs(input) - RobotConstants.JOYSTICK_DEAD_ZONE) / (1.0 - RobotConstants.JOYSTICK_DEAD_ZONE);
    }

    private double applyInputCurve(double input) {
        if (input == 0) return 0;
        double sign = Math.signum(input);
        double magnitude = Math.abs(input);
        return sign * Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
    }

    private static double wrapRadians(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(STRAFE_POD_X, FORWARD_POD_Y, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }
}


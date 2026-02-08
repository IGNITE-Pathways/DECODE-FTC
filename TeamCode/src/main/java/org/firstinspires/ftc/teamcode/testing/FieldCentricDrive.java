package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

/**
 * Field-Centric Drive TeleOp using GoBilda Pinpoint Odometry Computer
 *
 * Robot moves relative to the FIELD, not the robot orientation.
 * Push forward = robot moves away from driver, regardless of robot heading.
 *
 * CONTROLS:
 *   Left Stick  = Move (field-centric)
 *   Right Stick = Rotate
 *   LB          = Toggle slow mode (35%)
 *   RB          = Toggle fast mode (95%)
 *   Y           = Reset heading (face away from driver)
 *   A           = Recalibrate IMU (robot must be still)
 */
@TeleOp(name = "Field Centric Drive", group = "Competition")
public class FieldCentricDrive extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Pinpoint odometry computer
    private GoBildaPinpointDriver pinpoint;

    // Pinpoint configuration (match your robot)
    private static final double FORWARD_POD_Y = 3.75;      // Y offset in inches
    private static final double STRAFE_POD_X = -7.08661;   // X offset in inches

    // Heading tracking
    private double headingOffset = 0;

    // Speed mode: 0 = normal (65%), 1 = slow (35%), 2 = fast (95%)
    private int speedMode = 0;

    // Button states for toggle
    private boolean lastLB = false;
    private boolean lastRB = false;
    private boolean lastY = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_FRONT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_FRONT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_BACK_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_BACK_MOTOR);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
        configurePinpoint();

        telemetry.addLine("=== FIELD CENTRIC DRIVE ===");
        telemetry.addLine("Pinpoint: OK");
        telemetry.addLine();
        telemetry.addLine("Y = Reset heading");
        telemetry.addLine("A = Recalibrate IMU (stay still)");
        telemetry.addLine("LB = Slow | RB = Fast");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Pinpoint
            pinpoint.update();

            // Get joystick inputs
            double y = -gamepad1.left_stick_y;  // Forward/back (inverted)
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset heading with Y button
            if (gamepad1.y && !lastY) {
                resetHeading();
            }
            lastY = gamepad1.y;

            // Recalibrate IMU with A button (robot must be still)
            if (gamepad1.a && !lastA) {
                pinpoint.resetPosAndIMU();
                headingOffset = 0;
            }
            lastA = gamepad1.a;

            // Speed mode toggles
            if (gamepad1.left_bumper && !lastLB) {
                speedMode = (speedMode == 1) ? 0 : 1;
            }
            if (gamepad1.right_bumper && !lastRB) {
                speedMode = (speedMode == 2) ? 0 : 2;
            }
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            // Get robot heading from Pinpoint (in radians for trig)
            Pose2D pose = pinpoint.getPosition();
            double botHeading = Math.toRadians(pose.getHeading(AngleUnit.DEGREES) - headingOffset);

            // Field-centric transformation
            // Rotate the movement direction by the inverse of the robot's heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Slight strafe correction (mecanum wheels strafe slower than drive)
            rotX = rotX * 1.1;

            // Calculate motor powers (mecanum drive)
            double frontLeftPower = rotY + rotX + rx;
            double backLeftPower = rotY - rotX + rx;
            double frontRightPower = rotY - rotX - rx;
            double backRightPower = rotY + rotX - rx;

            // Normalize motor powers
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            // Apply speed multiplier
            double speedMult = (speedMode == 1) ? 0.35 : (speedMode == 2) ? 0.95 : 0.65;
            frontLeftPower *= speedMult;
            backLeftPower *= speedMult;
            frontRightPower *= speedMult;
            backRightPower *= speedMult;

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry
            telemetry.addLine("=== FIELD CENTRIC DRIVE ===");
            telemetry.addData("Speed", speedMode == 1 ? "SLOW 35%" : speedMode == 2 ? "FAST 95%" : "NORMAL 65%");
            telemetry.addData("Heading", "%.1f°", pose.getHeading(AngleUnit.DEGREES) - headingOffset);
            telemetry.addLine();
            telemetry.addLine("--- PINPOINT ---");
            telemetry.addData("X", "%.1f in", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", "%.1f in", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addLine();
            telemetry.addLine("Y = Reset heading");
            telemetry.addLine("A = Recalibrate IMU (stay still)");
            telemetry.addLine("LB = Slow | RB = Fast");
            telemetry.update();
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void configurePinpoint() {
        // Set odometry pod offsets (relative to center of robot)
        pinpoint.setOffsets(STRAFE_POD_X, FORWARD_POD_Y, DistanceUnit.INCH);

        // Set encoder type (GoBilda 4-bar pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();
    }

    private void resetHeading() {
        Pose2D pose = pinpoint.getPosition();
        headingOffset = pose.getHeading(AngleUnit.DEGREES);
    }
}

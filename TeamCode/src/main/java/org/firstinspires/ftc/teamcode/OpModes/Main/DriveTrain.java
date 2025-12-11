package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

@TeleOp(name = "DriveTrain", group = "Main")
public class DriveTrain extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private boolean crossPressedLast = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_FRONT_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_FRONT_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_BACK_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_BACK_MOTOR);

        // Correct motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check for cross button (special maneuver)
            if (gamepad1.cross && !crossPressedLast) {
                turnAndMoveBackwardMecanum(135, 0.6, 2000);
            }
            crossPressedLast = gamepad1.cross;

            // Standard mecanum driving
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Mecanum wheel math
            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backLeftPower = forward - right + rotate;
            double backRightPower = forward + right - rotate;

            // Normalize
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Apply powers
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            telemetry.update();
        }
    }

    public void turnAndMoveBackwardMecanum(double angleDegrees, double power, long durationMs) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double initialTurnPower = angleDegrees / 180.0;

        while (timer.milliseconds() < durationMs && opModeIsActive()) {
            double t = timer.milliseconds() / (double) durationMs;
            double rotate = -initialTurnPower * (1.0 - t);
            double forward = -power;

            double frontLeftPower = forward + rotate;
            double frontRightPower = forward - rotate;
            double backLeftPower = forward + rotate;
            double backRightPower = forward - rotate;

            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
        }

        // Stop motors
        stopMotors();
    }

    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
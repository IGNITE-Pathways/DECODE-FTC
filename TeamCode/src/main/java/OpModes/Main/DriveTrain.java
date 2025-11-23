package OpModes.Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// ✓ Helper class - does NOT extend OpMode
public class DriveTrain {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private Gamepad gamepad1;
    private boolean crossPressedLast = false;

    // Constructor accepts hardwareMap and gamepad
    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftbackmotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightbackmotor");

        // Correct motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    // Call this in your main loop
    public void loop() {
        // Check for cross button (special maneuver)
        if (gamepad1.cross && !crossPressedLast) {
            turnAndMoveBackwardMecanum(135, 0.6, 2000);
        }
        crossPressedLast = gamepad1.cross;

        // Standard mecanum driving
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x * 1.1; // strafe correction
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
    }

    public void turnAndMoveBackwardMecanum(double angleDegrees, double power, long durationMs) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double initialTurnPower = angleDegrees / 180.0;

        while (timer.milliseconds() < durationMs) {
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
package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Transfer Ramp Servo Test
 *
 * CONTROLS:
 *   DPAD UP      = Move up (+0.05)
 *   DPAD DOWN    = Move down (-0.05)
 */
@TeleOp(name = "Test: Transfer Ramp", group = "Test")
public class TransferRampTest extends LinearOpMode {

    private Servo transferServo;
    private double currentPosition = 0.5;

    // Edge detection
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        // Initialize servo
        transferServo = hardwareMap.get(Servo.class, HardwareConfig.TRANSFER_SERVO);
        transferServo.setPosition(currentPosition);

        telemetry.addLine("Transfer Ramp Test");
        telemetry.addLine("Dpad Up/Down to adjust");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust position
            if (gamepad1.dpad_up && !prevDpadUp) {
                currentPosition = Math.min(1.0, currentPosition + 0.05);
                transferServo.setPosition(currentPosition);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                currentPosition = Math.max(0.0, currentPosition - 0.05);
                transferServo.setPosition(currentPosition);
            }

            // Update edge detection
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // Telemetry
            telemetry.addData("Servo Position", "%.3f", currentPosition);
            telemetry.addLine();
            telemetry.addLine("Dpad Up = +0.05");
            telemetry.addLine("Dpad Down = -0.05");
            telemetry.update();

            idle();
        }
    }
}

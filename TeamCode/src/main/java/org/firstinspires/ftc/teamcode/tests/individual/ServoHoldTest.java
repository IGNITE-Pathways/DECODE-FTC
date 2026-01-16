package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

@TeleOp(name = "Servo Hold Test", group = "Tests")
public class ServoHoldTest extends LinearOpMode {

    private Servo transferServo;
    private double commandedPosition = 0.5;
    private boolean positionSet = false;
    private long positionSetTime = 0;

    @Override
    public void runOpMode() {
        try {
            transferServo = hardwareMap.get(Servo.class, HardwareConfig.TRANSFER_SERVO);
            telemetry.addLine("✓ Transfer servo found");
        } catch (Exception e) {
            telemetry.addLine("✗ Servo not found!");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("=================================");
        telemetry.addLine("  SERVO HOLD TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("This test will:");
        telemetry.addLine("1. Command the servo to 0.5 ONCE");
        telemetry.addLine("2. NEVER send another command");
        telemetry.addLine("3. Monitor if position changes");
        telemetry.addLine();
        telemetry.addLine("If the servo keeps moving,");
        telemetry.addLine("it's likely BROKEN or has");
        telemetry.addLine("electrical problems.");
        telemetry.addLine();
        telemetry.addLine("Press START...");
        telemetry.update();

        waitForStart();

        // Set position ONCE at start
        commandedPosition = 0.5;
        transferServo.setPosition(commandedPosition);
        positionSet = true;
        positionSetTime = System.currentTimeMillis();

        telemetry.addLine("Position set to 0.5");
        telemetry.addLine("Now monitoring...");
        telemetry.update();
        sleep(1000);

        double previousPosition = transferServo.getPosition();
        int unchangedCount = 0;
        int changedCount = 0;

        while (opModeIsActive()) {
            // DO NOT SEND ANY COMMANDS - just monitor
            double currentPosition = transferServo.getPosition();
            long elapsedSeconds = (System.currentTimeMillis() - positionSetTime) / 1000;

            double positionChange = Math.abs(currentPosition - previousPosition);

            if (positionChange > 0.001) {
                changedCount++;
            } else {
                unchangedCount++;
            }

            // Optional manual control to test different positions
            boolean manualCommand = false;
            if (gamepad1.a) {
                commandedPosition = 0.0;
                transferServo.setPosition(commandedPosition);
                positionSetTime = System.currentTimeMillis();
                manualCommand = true;
                changedCount = 0;
                unchangedCount = 0;
                sleep(200);
            } else if (gamepad1.b) {
                commandedPosition = 0.5;
                transferServo.setPosition(commandedPosition);
                positionSetTime = System.currentTimeMillis();
                manualCommand = true;
                changedCount = 0;
                unchangedCount = 0;
                sleep(200);
            } else if (gamepad1.y) {
                commandedPosition = 1.0;
                transferServo.setPosition(commandedPosition);
                positionSetTime = System.currentTimeMillis();
                manualCommand = true;
                changedCount = 0;
                unchangedCount = 0;
                sleep(200);
            }

            telemetry.clear();
            telemetry.addLine("=================================");
            telemetry.addLine("  SERVO HOLD TEST - MONITORING");
            telemetry.addLine("=================================");
            telemetry.addLine();

            telemetry.addData("Commanded Position", "%.3f", commandedPosition);
            telemetry.addData("Actual Position", "%.3f", currentPosition);
            telemetry.addData("Position Error", "%.3f", Math.abs(commandedPosition - currentPosition));
            telemetry.addLine();

            telemetry.addData("Time Since Command", "%d seconds", elapsedSeconds);
            telemetry.addData("Position Changes", changedCount);
            telemetry.addData("Position Stable", unchangedCount);
            telemetry.addLine();

            // Diagnosis
            telemetry.addLine("=================================");
            telemetry.addLine("DIAGNOSIS:");
            telemetry.addLine("=================================");

            if (elapsedSeconds > 3) {
                if (changedCount > unchangedCount * 0.1) {
                    telemetry.addLine("⚠ SERVO IS MOVING!");
                    telemetry.addLine();
                    telemetry.addLine("Possible causes:");
                    telemetry.addLine("1. Servo is BROKEN/DAMAGED");
                    telemetry.addLine("2. Power supply issue");
                    telemetry.addLine("3. Servo hitting mechanical limit");
                    telemetry.addLine("4. Wrong servo (is it 180° servo?)");
                    telemetry.addLine();
                    telemetry.addLine("SOLUTION:");
                    telemetry.addLine("- Try a different servo");
                    telemetry.addLine("- Check for mechanical binding");
                    telemetry.addLine("- Verify servo type (should be 180°)");
                } else {
                    telemetry.addLine("✓ SERVO IS HOLDING POSITION");
                    telemetry.addLine();
                    telemetry.addLine("Servo appears to be working");
                    telemetry.addLine("correctly. If you're seeing");
                    telemetry.addLine("movement in other programs,");
                    telemetry.addLine("check the code for repeated");
                    telemetry.addLine("setPosition() calls.");
                }
            } else {
                telemetry.addLine("Waiting for servo to settle...");
                telemetry.addLine("(first 3 seconds)");
            }

            telemetry.addLine();
            telemetry.addLine("=================================");
            telemetry.addLine("MANUAL TEST:");
            telemetry.addData("A", "Set to 0.0");
            telemetry.addData("B", "Set to 0.5");
            telemetry.addData("Y", "Set to 1.0");
            telemetry.addLine("=================================");

            telemetry.update();

            previousPosition = currentPosition;
            sleep(100);
        }
    }
}

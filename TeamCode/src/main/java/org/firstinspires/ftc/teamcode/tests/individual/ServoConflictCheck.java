package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

@TeleOp(name = "Servo Conflict Check", group = "Tests")
public class ServoConflictCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo transferServo = null;

        telemetry.addLine("=================================");
        telemetry.addLine("  CHECKING FOR CONFLICTS");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("BEFORE pressing START:");
        telemetry.addLine("- Is the servo moving already?");
        telemetry.addLine("- Is it beeping?");
        telemetry.addLine();
        telemetry.addLine("If YES: Another program is running");
        telemetry.addLine("or there's a hardware issue.");
        telemetry.addLine();
        telemetry.addLine("If NO: Press START to continue");
        telemetry.update();

        // Wait a long time to observe
        sleep(5000);

        try {
            transferServo = hardwareMap.get(Servo.class, HardwareConfig.TRANSFER_SERVO);
            telemetry.addLine("✓ Servo found in hardware map");
        } catch (Exception e) {
            telemetry.addLine("✗ Servo NOT found!");
            telemetry.addLine("Error: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        double initialPosition = transferServo.getPosition();

        telemetry.clear();
        telemetry.addLine("=================================");
        telemetry.addLine("  SERVO INITIAL STATE");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addData("Initial Position", "%.3f", initialPosition);
        telemetry.addLine();
        telemetry.addLine("Watching for 3 seconds...");
        telemetry.addLine("(NOT sending any commands)");
        telemetry.update();

        // Watch for changes WITHOUT sending commands
        long startTime = System.currentTimeMillis();
        double maxChange = 0;
        double previousPos = initialPosition;
        int changeCount = 0;

        while (System.currentTimeMillis() - startTime < 3000) {
            double currentPos = transferServo.getPosition();
            double change = Math.abs(currentPos - previousPos);

            if (change > 0.001) {
                changeCount++;
                if (change > maxChange) {
                    maxChange = change;
                }
            }

            previousPos = currentPos;
            sleep(50);
        }

        telemetry.clear();
        telemetry.addLine("=================================");
        telemetry.addLine("  OBSERVATION RESULTS");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addData("Position changes detected", changeCount);
        telemetry.addData("Max change per reading", "%.3f", maxChange);
        telemetry.addLine();

        if (changeCount > 5) {
            telemetry.addLine("⚠ SERVO IS MOVING!");
            telemetry.addLine();
            telemetry.addLine("Possible causes:");
            telemetry.addLine("1. Another OpMode is running");
            telemetry.addLine("2. Servo is continuous rotation");
            telemetry.addLine("3. Servo is broken/faulty");
            telemetry.addLine("4. Electrical interference");
            telemetry.addLine("5. PWM signal issue");
            telemetry.addLine();
            telemetry.addLine("STOP ALL OTHER OPMODES!");
        } else {
            telemetry.addLine("✓ Servo is stable (not moving)");
            telemetry.addLine();
            telemetry.addLine("Ready to test manual control.");
        }

        telemetry.addLine();
        telemetry.addLine("Press START to test control...");
        telemetry.update();

        waitForStart();

        // Now try to control it
        telemetry.clear();
        telemetry.addLine("=================================");
        telemetry.addLine("  TESTING MANUAL CONTROL");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("Commanding servo to 0.3...");
        telemetry.update();

        transferServo.setPosition(0.3);
        sleep(1000);

        telemetry.addLine("Commanding servo to 0.7...");
        telemetry.update();

        transferServo.setPosition(0.7);
        sleep(1000);

        telemetry.addLine("Commanding servo to 0.5...");
        telemetry.update();

        transferServo.setPosition(0.5);
        sleep(1000);

        double finalPos = transferServo.getPosition();

        telemetry.clear();
        telemetry.addLine("=================================");
        telemetry.addLine("  TEST COMPLETE");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addData("Commanded position", "0.5");
        telemetry.addData("Actual position", "%.3f", finalPos);
        telemetry.addData("Error", "%.3f", Math.abs(0.5 - finalPos));
        telemetry.addLine();

        if (Math.abs(0.5 - finalPos) < 0.05) {
            telemetry.addLine("✓ Servo responding correctly");
        } else {
            telemetry.addLine("⚠ Servo not at commanded position");
            telemetry.addLine("Check servo type in config!");
        }

        telemetry.addLine();
        telemetry.addLine("Monitoring position...");
        telemetry.addLine("(No more commands being sent)");
        telemetry.update();

        // Monitor without sending commands
        while (opModeIsActive()) {
            double pos = transferServo.getPosition();

            telemetry.addData("Current Position", "%.3f", pos);
            telemetry.addData("Drift from 0.5", "%.3f", Math.abs(0.5 - pos));

            if (Math.abs(0.5 - pos) > 0.1) {
                telemetry.addLine();
                telemetry.addLine("⚠ POSITION DRIFTING!");
                telemetry.addLine("Servo may be broken or");
                telemetry.addLine("continuous rotation type!");
            }

            telemetry.update();
            sleep(100);
        }
    }
}

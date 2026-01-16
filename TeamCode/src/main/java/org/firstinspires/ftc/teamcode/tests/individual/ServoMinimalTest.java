package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

@TeleOp(name = "Servo Minimal Test", group = "Tests")
public class ServoMinimalTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo transferServo = null;

        try {
            transferServo = hardwareMap.get(Servo.class, HardwareConfig.TRANSFER_SERVO);
            telemetry.addLine("✓ Servo found");
        } catch (Exception e) {
            telemetry.addLine("✗ Servo not found!");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("=================================");
        telemetry.addLine("  MINIMAL SERVO TEST");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("This test does NOTHING until you");
        telemetry.addLine("press START. After START, it will");
        telemetry.addLine("set servo to 0.5 ONCE and stop.");
        telemetry.addLine();
        telemetry.addLine("If servo moves BEFORE you press");
        telemetry.addLine("START, it's a hardware issue.");
        telemetry.addLine();
        telemetry.addLine("Press START when ready...");
        telemetry.update();

        // DO NOTHING - just wait
        waitForStart();

        // After start, set position ONCE
        telemetry.addLine("Setting servo to 0.5...");
        telemetry.update();

        transferServo.setPosition(0.5);

        sleep(2000);

        telemetry.addLine("Done. Servo should stay at 0.5");
        telemetry.addLine("Press STOP to end.");
        telemetry.update();

        // Loop but do NOTHING
        while (opModeIsActive()) {
            // Intentionally empty - no commands sent
            sleep(100);
        }
    }
}

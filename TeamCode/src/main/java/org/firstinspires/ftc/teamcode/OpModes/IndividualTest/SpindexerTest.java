package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Spindexer;

@TeleOp(name = "Test: Spindexer (Positional 720°)", group = "Test")
public class SpindexerTest extends LinearOpMode {

    private Spindexer spindexer;

    private boolean prevA = false;

    // 720° sail-winch style servo
    private static final double STEP_DEGREES = 60.0;   // one press = +60°

    @Override
    public void runOpMode() {

        // Read servo position BEFORE component initialization to preserve original position
        Servo tempServo = hardwareMap.get(Servo.class, "indexServo");
        double originalPos = tempServo != null ? tempServo.getPosition() : 1;
        double targetDegrees = originalPos * Spindexer.MAX_DEGREES;
        targetDegrees = Spindexer.clipDeg(targetDegrees);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        // Restore original servo position (component initialized it to 685°, but we want original)
        spindexer.setPositionDirect(targetDegrees);

        telemetry.addLine("Ready. Press A to move +60° (positional 720° servo).");

        addTelemetry();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;

            if (a && !prevA) {
                // increment 60°
                spindexer.incrementPosition(-STEP_DEGREES);
            }

            prevA = a;

            addTelemetry();
            telemetry.update();
            idle();

        }

    }

    private void addTelemetry() {
        telemetry.addData("Target°", "%.1f / %.0f", spindexer.getTargetDegrees(), Spindexer.MAX_DEGREES);
        telemetry.addData("Servo pos", "%.3f", spindexer.getServoPosition());
        telemetry.addLine("A: +60°   (adjust STEP_DEGREES to change increment)");
    }
}

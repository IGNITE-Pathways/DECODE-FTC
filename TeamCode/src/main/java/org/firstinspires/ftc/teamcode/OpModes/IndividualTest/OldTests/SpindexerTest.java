package org.firstinspires.ftc.teamcode.OpModes.IndividualTest.OldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Main.Components.OldComponents.Spindexer;

@TeleOp(name = "Test: Spindexer", group = "Test")
public class SpindexerTest extends LinearOpMode {

    private Spindexer spindexer;

    private boolean prevA = false;

    private boolean prevB = false;

    // 720° sail-winch style servo
    private static final double STEP_DEGREES = 60.0;   // one press = +60°

    @Override
    public void runOpMode() {
        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        telemetry.addLine("Ready. Press A to move +60° (positional 720° servo).");

        addTelemetry();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !prevA) {
                // increment q60°


                spindexer.smoothMoveTo(spindexer.getTargetDegrees() + 5);
            }

            prevA = a;

            if (b && !prevB){
                spindexer.kick();
            }
            prevB = b;


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

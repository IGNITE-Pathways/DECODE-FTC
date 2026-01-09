package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Main.Components.IntakeTransfer;

/**
 * Test OpMode for the IntakeTransfer mechanism.
 *
 * CONTROLS:
 * - RT (Right Trigger): Run intake motor (variable speed)
 * - LT (Left Trigger): Reverse intake / eject balls
 * - B: Move transfer ramp UP (transfer to shooter)
 * - X: Move transfer ramp DOWN (ready position)
 *
 * This is a standalone test - no other robot components required.
 */
@TeleOp(name = "Test: IntakeTransfer", group = "Test")
public class IntakeTransferTest extends LinearOpMode {

    private IntakeTransfer intakeTransfer;

    @Override
    public void runOpMode() {
        // Initialize the component
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        // Display controls on init
        telemetry.addLine("=== IntakeTransfer Test ===");
        telemetry.addLine("");
        telemetry.addLine("INTAKE MOTOR:");
        telemetry.addLine("  RT = Run intake");
        telemetry.addLine("  LT = Eject / reverse");
        telemetry.addLine("");
        telemetry.addLine("TRANSFER RAMP:");
        telemetry.addLine("  B = Ramp UP (transfer)");
        telemetry.addLine("  X = Ramp DOWN (ready)");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update component with gamepad input
            intakeTransfer.update(gamepad1);

            // Display status
            telemetry.addLine("=== IntakeTransfer Status ===");
            telemetry.addLine("");
            telemetry.addLine("Controls: RT/LT = motor, B/X = ramp");
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.OldComponents.Intake;

@TeleOp(name = "Test: Intake", group = "Test")
public class IntakeTest extends LinearOpMode {

    private Intake intake;

    @Override
    public void runOpMode() {
        intake = new Intake();
        intake.initialize(hardwareMap, telemetry);

        telemetry.addLine("Intake Test Initialized");
        telemetry.addLine("Cross: Start intake");
        telemetry.addLine("Circle: Stop intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Start intake on Cross
            if (gamepad1.cross) {
                intake.start();
            }

            // Stop intake on Circle
            if (gamepad1.circle) {
                intake.stop();
            }

            // Update intake (updates telemetry)
            intake.update();

            telemetry.addData("Intake Power", "%.2f", intake.getPower());
            telemetry.addData("Intake Status", intake.isRunning() ? "Running" : "Stopped");
            telemetry.update();
        }
    }
}

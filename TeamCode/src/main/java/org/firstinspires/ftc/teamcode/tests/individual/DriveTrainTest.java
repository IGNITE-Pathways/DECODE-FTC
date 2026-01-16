package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
@Disabled
@TeleOp(name="Test: DriveTrain", group="Test")
public class DriveTrainTest extends OpMode {

    private DriveTrain driveTrain;

    @Override
    public void init() {
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);
        telemetry.addLine("DriveTrain Test Initialized");
        telemetry.addLine("Left Stick: Move");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("Cross: Precision mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get gamepad inputs
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Update drive train
        driveTrain.update(forward, right, rotate);

        // Telemetry
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Right", "%.2f", right);
        telemetry.addData("Rotate", "%.2f", rotate);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop motors when OpMode stops
        driveTrain.stopMotors();
    }
}

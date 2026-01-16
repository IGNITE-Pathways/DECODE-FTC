package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.components.Flywheel;
@Disabled
@TeleOp(name="Test: FlyWheelTest", group="Test")
public class FlyWheelTest extends OpMode {

    private Flywheel flywheel;

    // Edge detection variables
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevSquare = false;
    private boolean prevCircle = false;

    @Override
    public void init() {
        flywheel = new Flywheel();
        flywheel.initialize(hardwareMap, telemetry, 0.1);
    }

    @Override
    public void loop() {
        // Increase power with Right Bumper (edge detection)
        if (gamepad1.right_bumper && !prevRightBumper) {
            flywheel.adjustPower(0.1);
        }

        // Decrease power with Left Bumper (edge detection)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            flywheel.adjustPower(-0.1);
        }

        // Start spinning with Square (edge detection)
        if (gamepad1.square && !prevSquare) {
            flywheel.setSpinning(true);
        }

        // Stop spinning with Circle (edge detection)
        if (gamepad1.circle && !prevCircle) {
            flywheel.setSpinning(false);
        }

        // Update previous button states for edge detection
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        prevSquare = gamepad1.square;
        prevCircle = gamepad1.circle;

        // Update flywheel motors
        flywheel.update();

        // Telemetry
        telemetry.addData("Flywheel Power", "%.2f", flywheel.getPower());
        telemetry.addData("Status", flywheel.isSpinning() ? "Shooting" : "Stopped");
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.IndividualTest.OldTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Launcher;

@TeleOp(name="Test: Launcher", group="Test")
public class LauncherTest extends OpMode {

    private Launcher launcher;

    // Edge detection variables
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevSquare = false;
    private boolean prevCircle = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void init() {
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);
        telemetry.addLine("Launcher Test Initialized");
        telemetry.addLine("Right Bumper: Increase power");
        telemetry.addLine("Left Bumper: Decrease power");
        telemetry.addLine("Square: Start flywheel");
        telemetry.addLine("Circle: Stop flywheel");
        telemetry.addLine("D-Pad Up: Increment hood");
        telemetry.addLine("D-Pad Down: Decrement hood");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Increase power with Right Bumper (edge detection)
        if (gamepad1.right_bumper && !prevRightBumper) {
            launcher.setPower(Math.min(1.0, launcher.getPower() + 0.05));
        }

        // Decrease power with Left Bumper (edge detection)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            launcher.setPower(Math.max(0.0, launcher.getPower() - 0.05));
        }

        // Start spinning with Square (edge detection)
        if (gamepad1.square && !prevSquare) {
            launcher.setSpinning(true);
        }

        // Stop spinning with Circle (edge detection)
        if (gamepad1.circle && !prevCircle) {
            launcher.setSpinning(false);
        }

        // Increment hood with D-Pad Up (edge detection)
        if (gamepad1.dpad_up && !prevDpadUp) {
            launcher.incrementHood();
        }

        // Decrement hood with D-Pad Down (edge detection)
        if (gamepad1.dpad_down && !prevDpadDown) {
            launcher.decrementHood();
        }

        // Update previous button states for edge detection
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        prevSquare = gamepad1.square;
        prevCircle = gamepad1.circle;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        // Update launcher (flywheel motors)
        launcher.update();

        // Telemetry
        telemetry.addData("Flywheel Power", "%.2f", launcher.getPower());
        telemetry.addData("Flywheel Status", launcher.isSpinning() ? "Spinning" : "Stopped");
        telemetry.addData("Hood Position", "%.3f", launcher.getHoodPosition());
        telemetry.update();
    }
}

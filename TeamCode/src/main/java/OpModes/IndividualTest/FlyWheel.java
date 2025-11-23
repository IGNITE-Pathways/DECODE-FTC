package OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ProgrammingBoard.ProgrammingBoardOTHER;

@TeleOp(name="FlyWheel Individual Control", group="Individual Test")
public class FlyWheel extends OpMode {

    ProgrammingBoardOTHER board = new ProgrammingBoardOTHER();

    private double flywheelPower = 0.1; // starting power
    private boolean spinning = false;   // flywheel state

    @Override
    public void init() {
        board.initializeComponents(hardwareMap);
        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Increase power with Triangle (Y)
        if (gamepad1.triangle) {
            flywheelPower += 0.1;
        }

        // Decrease power with X
        if (gamepad1.cross) {
            flywheelPower -= 0.1;
        }

        // Clamp power between 0 and 1
        flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));

        // Start spinning with Square
        if (gamepad1.square) {
            spinning = true;
        }

        // Stop spinning with Circle
        if (gamepad1.circle) {
            spinning = false;
        }

        // Apply power to flywheel

        board.flyWheelMotor.setPower(spinning ? flywheelPower : 0);

        board.flyWheelMotor2.setPower(spinning ? flywheelPower : 0);

        // Telemetry
        telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
        telemetry.addData("Status", spinning ? "Shooting" : "Stopped");
        telemetry.update();

        // Small delay to prevent button bounce
        sleep(160);
    }

    // Helper sleep function
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

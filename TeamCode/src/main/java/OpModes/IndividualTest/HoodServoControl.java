package OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hood Servo Individual Control", group="Individual Test")
public class HoodServoControl extends OpMode {

    private Servo hoodServo;
    private double servoPosition = 0.0; // Initial position (adjust as needed)
    private final double INCREMENT = 0.1;

    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodservo");
        hoodServo.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized. Servo position: %.2f", servoPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Move servo backward on X
        if (gamepad1.cross) {
            servoPosition -= INCREMENT;
        }

        // Move servo forward on Triangle (Y)
        if (gamepad1.triangle) {
            servoPosition += INCREMENT;
        }

        // Clamp servo position between 0.0 and 1.0
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Apply the position
        hoodServo.setPosition(servoPosition);

        // Telemetry
        telemetry.addData("Servo Position", "%.2f", servoPosition);
        telemetry.update();

        // Debounce delay
        sleep(120);
    }

    // Debounce helper method
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

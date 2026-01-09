package org.firstinspires.ftc.teamcode.OpModes.IndividualTest.OldTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test: Kicker", group="Test")
public class KickerTest extends OpMode {

    private Servo kickerServo;
    private double servoPosition;      // Current servo position
    private boolean goingUp = true;    // Flag to track direction
    private boolean lastTriangle = false; // For edge detection

    private final double MAX_POS = 0.7; // ~2 degrees in/ normalized servo units
    private final double MIN_POS = 0.011;   //

    @Override
    public void init() {
        kickerServo = hardwareMap.get(Servo.class, "kicker"); // Make sure name matches hardware

        servoPosition = MIN_POS;
        kickerServo.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized. Servo at %.2f", servoPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Detect rising edge of triangle button
        if (gamepad1.triangle && !lastTriangle) {
            if (goingUp) {
                servoPosition = MAX_POS;
                goingUp = false; // Next press will go down
            } else {
                servoPosition = MIN_POS;
                goingUp = true;  // Next press will go up
            }

            kickerServo.setPosition(servoPosition);
            telemetry.addData("Servo Position", "%.2f", servoPosition);
            telemetry.addData("Direction Flag", goingUp ? "Up next" : "Down next");
            telemetry.update();
        }

        lastTriangle = gamepad1.triangle; // Update edge detection
    }

}

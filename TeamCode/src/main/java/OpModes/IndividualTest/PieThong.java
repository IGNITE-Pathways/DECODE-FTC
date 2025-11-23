package OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Full Servo Hardware Test", group = "Test")
public class PieThong extends LinearOpMode {

    private Servo testServo = null;
    private CRServo testCRServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing servo test...");
        telemetry.update();

        // Try mapping as a positional servo first
        try {
            testServo = hardwareMap.get(Servo.class, "Pie1Servo");
            telemetry.addLine("Found standard Servo with name 'Pie1Servo'.");
        } catch (Exception e) {
            telemetry.addLine("No standard Servo found. Checking for CRServo...");
        }

        // Try mapping as a continuous rotation servo
        try {
            testCRServo = hardwareMap.get(CRServo.class, "Pie1Servo");
            telemetry.addLine("Found CRServo with name 'Pie1Servo'.");
        } catch (Exception e) {
            telemetry.addLine("No CRServo found either. Check your config name.");
        }

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // === STANDARD SERVO TEST ===
            if (testServo != null) {
                if (gamepad1.cross) testServo.setPosition(0.0);  // full left
                if (gamepad1.triangle) testServo.setPosition(0.5); // middle
                if (gamepad1.circle) testServo.setPosition(1.0);  // full right
            }

            // === CONTINUOUS ROTATION SERVO TEST ===
            if (testCRServo != null) {
                if (gamepad1.square) testCRServo.setPower(-1.0);  // full reverse
                else if (gamepad1.circle) testCRServo.setPower(1.0); // full forward
                else if (gamepad1.cross) testCRServo.setPower(0.0);  // stop
            }

            telemetry.addLine("==== Controls ====");
            telemetry.addLine("[For standard Servo]");
            telemetry.addLine("  ✖ (Cross) = 0.0, △ (Triangle) = 0.5, ◯ (Circle) = 1.0");
            telemetry.addLine("[For CRServo]");
            telemetry.addLine("  ◻ (Square) = reverse, ◯ (Circle) = forward, ✖ (Cross) = stop");
            telemetry.addLine("==================");
            telemetry.addData("Standard Servo Found?", testServo != null);
            telemetry.addData("CRServo Found?", testCRServo != null);
            telemetry.addData("Servo Position", testServo != null ? testServo.getPosition() : "N/A");
            telemetry.addData("CRServo Power", testCRServo != null ? testCRServo.getPower() : "N/A");
            telemetry.update();

            sleep(50); // give time for movement
        }
    }
}

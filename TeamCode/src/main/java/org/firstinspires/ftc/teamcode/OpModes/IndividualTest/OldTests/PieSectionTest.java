package org.firstinspires.ftc.teamcode.OpModes.IndividualTest.OldTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Test: PieSection", group = "Test")
public class PieSectionTest extends LinearOpMode {

    private Servo pieServo = null;

    // === Configuration ===
    private double divisionSpacing = 0.07246376811; // distance between pie slots
    private double currentPosition = 0.5;           // start from middle
    private boolean squarePressed = false;
    private boolean circlePressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pieServo = hardwareMap.get(Servo.class, "Pie1Servo");

        telemetry.addLine("Pie Stepper Test Initialized");
        telemetry.addData("Division Spacing", divisionSpacing);
        telemetry.addData("Start Position", currentPosition);
        telemetry.update();

        pieServo.setPosition(currentPosition);

        waitForStart();

        while (opModeIsActive()) {

            // === Move Right ===
            if (gamepad1.square && !squarePressed) {
                currentPosition += divisionSpacing;
                currentPosition = clamp(currentPosition, 0.0, 1.0);
                pieServo.setPosition(currentPosition);
                squarePressed = true;
            } else if (!gamepad1.square) {
                squarePressed = false; // reset when button released
            }

            // === Move Left ===
            if (gamepad1.circle && !circlePressed) {
                currentPosition -= divisionSpacing;
                currentPosition = clamp(currentPosition, 0.0, 1.0);
                pieServo.setPosition(currentPosition);
                circlePressed = true;
            } else if (!gamepad1.circle) {
                circlePressed = false;
            }

            telemetry.addLine("==== Controls ====");
            telemetry.addLine("◻ (Square): Move Right (next pie slot)");
            telemetry.addLine("◯ (Circle): Move Left (previous pie slot)");
            telemetry.addLine("==================");
            telemetry.addData("Servo Position", "%.4f", currentPosition);
            telemetry.addData("Division Spacing", divisionSpacing);
            telemetry.update();

            sleep(50); // small delay for stable input handling
        }
    }

    /**
     * Helper to keep servo position within valid range [0.0, 1.0].
     */
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

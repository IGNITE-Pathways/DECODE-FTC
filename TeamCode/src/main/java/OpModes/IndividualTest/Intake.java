package OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Intake Test", group = "Test")
public class Intake extends LinearOpMode {

    private CRServo centerServo; // This is still continuous

    @Override
    public void runOpMode() {
        // Map servos from hardware config
        centerServo = hardwareMap.get(CRServo.class, "ServoIntake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Start all on A
            if (gamepad1.cross) {
                centerServo.setPower(0.6);      // CRServo spins
            }

            // Stop all on B (circle)
            if (gamepad1.circle) {
                centerServo.setPower(0.0);      // Stop CRServo
            }

            telemetry.addData("Center Servo Power", centerServo.getPower());
            telemetry.update();
        }
    }
}

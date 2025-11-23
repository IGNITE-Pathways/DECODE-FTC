package OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import ProgrammingBoard.ProgrammingBoardOTHER;

@TeleOp(name = "SpindexerTest (Positional 720°)", group = "Linear OpMode")
public class SpindexerTest extends LinearOpMode {

    ProgrammingBoardOTHER board = new ProgrammingBoardOTHER();

    private Servo indexServo;

    private boolean prevA = false;

    // 720° sail-winch style servo
    private static final double MAX_DEGREES = 720.0;
    private static final double STEP_DEGREES = 60.0;   // one press = +60°


    // Track our target (servo only remembers last commanded position)
    private double targetDegrees = 0.0;

    @Override
    public void runOpMode() {

        board.initializeComponents(hardwareMap);

        // Ensure board.indexServo is a Servo in your ProgrammingBoardOTHER

        indexServo = board.indexServo;

        // Initialize target from current servo position (best-effort)
        targetDegrees = posToDeg(indexServo.getPosition());
        // Snap within [0, 720]

        targetDegrees = clampDeg(targetDegrees);

        // Go to starting target (keeps telemetry consistent)
        indexServo.setPosition(degToPos(targetDegrees));

        telemetry.addLine("Ready. Press A to move +60° (positional 720° servo).");

        addTelemetry();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;

            if (a && !prevA) {
                // increment 60°
                targetDegrees = clampDeg(targetDegrees + STEP_DEGREES);
                indexServo.setPosition(degToPos(targetDegrees));
            }

            prevA = a;

            addTelemetry();
            telemetry.update();
            idle();

        }

    }

    private void addTelemetry() {
        telemetry.addData("Target°", "%.1f / %.0f", targetDegrees, MAX_DEGREES);
        telemetry.addData("Servo pos", "%.3f", indexServo.getPosition());
        telemetry.addLine("A: +60°   (adjust STEP_DEGREES to change increment)");
    }

    // --- Helpers: angle <-> position mapping ---

    private static double degToPos(double degrees) {
        // 0..720°  ->  0.0..1.0
        return Range.clip(degrees / MAX_DEGREES, 0.0, 1.0);
    }

    private static double posToDeg(double pos) {
        // 0.0..1.0 -> 0..720°
        return Range.clip(pos, 0.0, 1.0) * MAX_DEGREES;
    }

    private static double clampDeg(double d) {
        return Range.clip(d, 0.0, MAX_DEGREES);
    }
}

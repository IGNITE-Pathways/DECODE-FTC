package org.firstinspires.ftc.teamcode.tests.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.tests.board.ProgrammingBoardShooter;

//@Disabled
@TeleOp(name = "Main Op Mode (CRServo Indexed)", group = "Linear OpMode")
public class IndexingOrShootPie extends LinearOpMode {

    ProgrammingBoardShooter board = new ProgrammingBoardShooter();
    private NormalizedColorSensor intakeColorSensor;
    private Servo indexServo; // ✅ New servo replacing pie motor/servo

    // Pie color pattern memory (3 slots)
    Map<Integer, String> indexColors = new HashMap<>();

    // Expected shoot order
    String[] need_colors = {"purple", "purple", "green"};

    int flag = 0; // index in need_colors
    int currentDivision = 0; // 0–2
    boolean imperfect = false;

    private boolean prevA = false;

    // 720° sail-winch style servo
    private static final double MAX_DEGREES = 720.0;
    private static final double STEP_DEGREES = 60.0;   // one press = +60°


    // Track our target (servo only remembers last commanded position)
    private double targetDegrees = 0.0;

    @Override
    public void runOpMode() {
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        indexServo = hardwareMap.get(Servo.class, "indexServo"); // ✅ renamed

        board.initializeComponents(hardwareMap);

        // Initialize stored pie colors (can start unknown)
        indexColors.put(0, "none");
        indexColors.put(1, "none");
        indexColors.put(2, "none");

        boolean isPurple = false;
        boolean isGreen = false;
        boolean intakeJustDetectedBall = false;


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
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            float hue = JavaUtil.colorToHue(colors.toColor());

            // === Intake color detection ===
            if (hue > 160 && hue < 350) { // purple
                isPurple = true;
                isGreen = false;
                intakeJustDetectedBall = true;
            } else if (hue >= 100 && hue <= 160) { // green
                isGreen = true;
                isPurple = false;
                intakeJustDetectedBall = true;
            } else {
                isPurple = false;
                isGreen = false;
            }


            // ===============================
            // Intake ball tracking logic
            // ===============================
            if (intakeJustDetectedBall) {
                // TODO: Check for intake being ran before confirming detection
                String detectedColor = isPurple ? "purple" : (isGreen ? "green" : "unknown");
                indexColors.put(currentDivision, detectedColor);
                telemetry.addData("Detected new ball:", detectedColor);
                intakeJustDetectedBall = false;
            }

            String neededBall = need_colors[flag];

            // ===========================
            // A → Manually move one division
            // ===========================

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
    private void addTelemetry () {
        telemetry.addData("Target°", "%.1f / %.0f", targetDegrees, MAX_DEGREES);
        telemetry.addData("Servo pos", "%.3f", indexServo.getPosition());
        telemetry.addLine("A: +60°   (adjust STEP_DEGREES to change increment)");
    }

    // --- Helpers: angle <-> position mapping ---

    private static double degToPos ( double degrees){
        // 0..720°  ->  0.0..1.0
        return Range.clip(degrees / MAX_DEGREES, 0.0, 1.0);
    }

    private static double posToDeg ( double pos){
        // 0.0..1.0 -> 0..720°
        return Range.clip(pos, 0.0, 1.0) * MAX_DEGREES;
    }

    private static double clampDeg ( double d){
        return Range.clip(d, 0.0, MAX_DEGREES);
    }
}
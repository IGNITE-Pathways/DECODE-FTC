package OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.HashMap;
import java.util.Map;


@TeleOp(name = "Main Spindexer", group = "Linear OpMode")
public class MainSpindexer extends LinearOpMode {
    int xPressCount = 0;  // counts how many balls have been shot
    private NormalizedColorSensor intakeColorSensor;
    private Servo indexServo;
    private CRServo intakeServo;

    Map<Integer, String> indexColors = new HashMap<>();

    String[] need_colors = {"purple", "purple", "green"};
    int flag = 0;

    int currentDivision = 0;

    boolean prevA = false;
    boolean prevB = false;
    boolean prevX = false;

    boolean prevY = false;

    private static final double MAX_DEGREES = 720.0;
    private static final double DIVISION_DEGREES = 55.0;

    private static final double SPEED_DEG_PER_STEP = 2.0;

    private double targetDegrees = 0.0;
    boolean intakeOn = false;
    @Override
    public void runOpMode() {
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        indexServo = hardwareMap.get(Servo.class, "indexServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");


        indexColors.put(0, "none");
        indexColors.put(1, "none");
        indexColors.put(2, "none");

        // Initialize exactly as you needed earlier:
        targetDegrees = MAX_DEGREES;
        indexServo.setPosition(posFromDeg(targetDegrees));

        telemetry.addLine("Ready. Press A to move 60°.");
        addTelemetry();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- COLOR DETECTION -----------------
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            float hue = JavaUtil.colorToHue(colors.toColor());

            boolean found = false;
            String detectedColor = "unknown";

            if (hue > 160 && hue < 350) {
                detectedColor = "purple";
                found = true;
            } else if (hue >= 100 && hue <= 160) {
                detectedColor = "green";
                found = true;
            }

            if (found) {
                indexColors.put(currentDivision, detectedColor);
                telemetry.addData("Detected new ball:", detectedColor);
                try {
                    // Pause execution for 3 seconds (3000 milliseconds)
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    // Handle the InterruptedException if the thread is interrupted while sleeping
                    e.printStackTrace();
                }
                rotateOneDivision();
            }

            // ---------------- BUTTON A -------------------------
            boolean Y = gamepad1.y;

            if (Y && !prevY){
                if (intakeOn){
                    intakeOn = false;
                    intakeServo.setPower(0);
                }
                if (!intakeOn){
                    intakeOn = true;
                    intakeServo.setPower(-0.6);
                }
            }
            prevY = Y;

            boolean A = gamepad1.a;
            if (A && !prevA) {
                targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
                smoothMoveTo(targetDegrees);
                currentDivision = (currentDivision + 1) % 3;
            }
            prevA = A;

            // ---------------- BUTTON B -------------------------
            boolean B = gamepad1.b;
            if (B && !prevB) {
                targetDegrees = MAX_DEGREES;
                smoothMoveTo(MAX_DEGREES);
                currentDivision = 0;
            }
            prevB = B;

            // ---------------- BUTTON X -------------------------

            boolean X = gamepad1.x;
            if (X && !prevX) {
                // Find the division of the next needed color
                int goalDiv = findDivisionWithColor(need_colors[(flag+2) % need_colors.length]);
                moveToDivision(goalDiv);  // move to that division first
                telemetry.addLine("Ball shot!");
                // "Shoot" the ball
                xPressCount++;
                flag++; // advance to next needed color

                // After 3 shots, reset everything
                if (xPressCount >= 3) {
                    // Reset HashMap
                    indexColors.put(0, "none");
                    indexColors.put(1, "none");
                    indexColors.put(2, "none");

                    // Reset servo
                    targetDegrees = MAX_DEGREES;
                    smoothMoveTo(targetDegrees);

                    // Reset counters
                    currentDivision = 0;
                    xPressCount = 0;
                    flag = 0;
                }
            }
            prevX = X;


            addTelemetry();
            telemetry.update();
            idle();
        }
    }

    // Telemetry helper
    private void addTelemetry() {
        telemetry.addData("Target°", "%.1f", targetDegrees);
        telemetry.addData("Servo Pos", "%.3f", indexServo.getPosition());
        telemetry.addData("Division", currentDivision);

        telemetry.addData("Div0", indexColors.get(0));
        telemetry.addData("Div1", indexColors.get(1));
        telemetry.addData("Div2", indexColors.get(2));
        telemetry.addData("Need Next", need_colors[flag]);

        telemetry.addLine("A = Move 1 division");
        telemetry.addLine("B = Reset");
    }

    // ---------------- SMOOTH SERVO MOVEMENT (FIXED) -------------------
    private void smoothMoveTo(double newTargetDeg) {

        // ❗️THIS FIXES THE ISSUE — start from actual servo output
        double startPos = indexServo.getPosition();  // 0.0–1.0
        double startDeg = startPos * MAX_DEGREES;

        double distance = newTargetDeg - startDeg;
        int steps = (int) Math.ceil(Math.abs(distance) / SPEED_DEG_PER_STEP);

        for (int i = 1; i <= steps; i++) {
            double interpDeg = startDeg + distance * (i / (double) steps);
            indexServo.setPosition(posFromDeg(interpDeg));
            sleep(10);
        }

        indexServo.setPosition(posFromDeg(newTargetDeg));
    }

    // ---------------- DIVISION ROTATION ---------------------
    private void rotateOneDivision() {
        currentDivision = (currentDivision + 1) % 3;
        targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
        smoothMoveTo(targetDegrees);
    }

    private void moveToDivision(int targetDiv) {
        int steps = (targetDiv - currentDivision + 3) % 3;
        for (int i = 0; i < steps; i++) {
            rotateOneDivision();
            sleep(150);
        }
    }

    private int findDivisionWithColor(String c) {
        for (int i = 0; i < 3; i++) {
            if (indexColors.get(i).equals(c)) return i;
        }
        return currentDivision;
    }

    // ---------------- DEG <-> POS HELPERS ---------------------
    private static double posFromDeg(double degrees) {
        return Range.clip(degrees / MAX_DEGREES, 0.0, 1.0);
    }

    private static double clipDeg(double d) {
        return Range.clip(d, 0.0, MAX_DEGREES);
    }
}

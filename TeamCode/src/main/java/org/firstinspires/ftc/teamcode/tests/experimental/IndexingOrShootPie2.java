package org.firstinspires.ftc.teamcode.tests.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.HashMap;
import java.util.Map;

//import org.firstinspires.ftc.teamcode.ProgrammingBoard.ProgrammingBoardOTHER;

@TeleOp(name = "Main Op Mode 2", group = "Linear OpMode")
public class IndexingOrShootPie2 extends LinearOpMode {

//    ProgrammingBoardOTHER board = new ProgrammingBoardOTHER();




    private NormalizedColorSensor intakeColorSensor;
    private Servo indexServo;
    private CRServo intakeServo;
    private CRServo kickerWheel;

    // Ball color storage (slot 0-2 -> color)
    // SLOT 0 = INTAKE POSITION (fixed)
    // SLOT 1 = MIDDLE POSITION (fixed)
    // SLOT 2 = LAUNCH POSITION (fixed)
    Map<Integer, String> ballColors = new HashMap<>();

    // Expected shoot order: purple, green, purple
    String[] need_colors = {"purple", "green", "purple"};
    int indexOFBALLNEEDEDTOBELAUNCHED = 0;

    // ⚙️ TUNE THESE VALUES IF ALIGNMENT IS OFF ⚙️
    private static final double MAX_DEGREES = 720.0;
    private static final double STEP_DEGREES = 55.25;   // One slot rotation
    private static final int SERVO_MOVE_TIME_MS = 600; // Time to wait for servo to complete movement

    private static final int NUM_SLOTS = 3;
    private static final int INTAKE_SLOT = 0;   // Fixed intake position
    private static final int LAST_SLOT = 1;   // Fixed middle position
    private static final int LAUNCH_SLOT = 2;   // Fixed launch position

    private static final double INITIAL_SERVO_POSITION = 675;

    private double targetDegrees = INITIAL_SERVO_POSITION;

    // Kicker config
    private static final double KICK_REST_POS = 1;
    private static final double KICK_FIRE_POS = 0.6;
    private static final int KICK_PULSE_MS = 1000;

    // Kicker wheel config
    private static final double KICKER_WHEEL_POWER = -1;
    private static final int KICKER_WHEEL_SPIN_TIME_MS = 1000;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevRightTrigger = false;

    private boolean intakeRunning = false;

    private String lastAction = "Initializing...";



    @Override
    public void runOpMode() {
//        board.initializeComponents(hardwareMap);


//        intakeColorSensor = board.intakeColorSensor;
//        indexServo = board.indexServo;
//        intakeServo = board.intakeServo;
//        kickerWheel = board.kickerWheel;

        // Initialize intake (OFF by default)
        intakeServo.setPower(0.0);
        intakeRunning = false;

        // Initialize kicker wheel (OFF by default)
        kickerWheel.setPower(0.0);

        // Initialize ball colors
        ballColors.put(INTAKE_SLOT, "none");   // Slot 0 - Intake
        ballColors.put(LAST_SLOT, "none");   // Slot 1 - Middle
        ballColors.put(LAUNCH_SLOT, "none");   // Slot 2 - Launch

        // Set servo to initial position DURING INIT
        targetDegrees = INITIAL_SERVO_POSITION;
        indexServo.setPosition(degreesToPosition(targetDegrees));


        lastAction = "Servo moving to " + INITIAL_SERVO_POSITION + "°...";

        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("   INITIALIZING SPINDEXER");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addData("Servo Target", "%.0f°", targetDegrees);
        telemetry.addData("Slot 0", "INTAKE");
        telemetry.addData("Slot 1", "MIDDLE");
        telemetry.addData("Slot 2", "LAUNCH");
        telemetry.update();

        sleep(1500);

        waitForStart();

        lastAction = "OpMode running";

        while (opModeIsActive()) {
            // Update color sensor reading


            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            float hue = JavaUtil.colorToHue(colors.toColor());

            // Handle button inputs
            handleButtonInputs(hue);

            // Display real-time telemetry
            displayTelemetry(hue);
            telemetry.update();
        }
    }

    /**
     * Handle all button inputs
     */
    private void handleButtonInputs(float hue) {
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;
        boolean x = gamepad2.x;
        boolean y = gamepad2.y;
        boolean rightTrigger = gamepad2.right_trigger > 0.5;

        // A button: rotate one slot clockwise (55.25°)
        if (a && !prevA) {
            double nextPos = targetDegrees - STEP_DEGREES;

            // Check if we would exceed range
            if (nextPos < 0) {
                double wrapped = nextPos + MAX_DEGREES;
                if (wrapped > MAX_DEGREES - STEP_DEGREES) {
                    lastAction = "⚠ AT MAX! Press Y to reset";
                } else {
                    rotateOneSlot();
                }
            } else {
                rotateOneSlot();
            }
        }

        // B button: kick ball at launch slot
        if (b && !prevB) {
            lastAction = "Kicking ball at LAUNCH (Slot 2)...";
            kickBall();
            ballColors.put(LAUNCH_SLOT, "none");  // Clear slot after kick
            lastAction = "Ball kicked from Slot 2 (LAUNCH)";
        }

        // X button: toggle intake on/off
        if (x && !prevX) {
            intakeRunning = !intakeRunning;
            intakeServo.setPower(intakeRunning ? 1.0 : 0.0);
            lastAction = "Intake: " + (intakeRunning ? "ON" : "OFF");
        }

        // Y button: reset to initial position
        if (y && !prevY) {
            lastAction = "Resetting to initial position...";
            resetToInitial();
        }

        // RIGHT TRIGGER: HOT BUTTON - Auto-align and launch needed ball
        if (rightTrigger && !prevRightTrigger) {
            autoLaunchNeededBall();
        }

        // Auto-detect and add balls at INTAKE SLOT (Slot 0)
        if (intakeRunning) {
            if (hue > 160 && hue < 350) {
                if (!ballColors.get(INTAKE_SLOT).equals("purple")) {
                    ballColors.put(INTAKE_SLOT, "purple");
                    lastAction = "✓ PURPLE detected → Slot 0 (INTAKE)";
                }
            } else if (hue >= 100 && hue <= 160) {
                if (!ballColors.get(INTAKE_SLOT).equals("green")) {
                    ballColors.put(INTAKE_SLOT, "green");
                    lastAction = "✓ GREEN detected → Slot 0 (INTAKE)";
                }
            }
        }

        prevA = a;
        prevB = b;
        prevX = x;
        prevY = y;
        prevRightTrigger = rightTrigger;
    }

    /**
     * HOT BUTTON: Auto-align needed ball to launch slot and fire
     */
    private void autoLaunchNeededBall() {
        String neededColor = need_colors[indexOFBALLNEEDEDTOBELAUNCHED];
        lastAction = "🔥 HOT BUTTON: Need " + neededColor.toUpperCase();

        // Find which slot has the needed ball
        int ballSlot = findSlotWithColor(neededColor);

        if (ballSlot == -1) {
            lastAction = "❌ ERROR: " + neededColor.toUpperCase() + " not found!";
            return;
        }

        // Calculate rotations needed to get ball to LAUNCH_SLOT (Slot 2)
        int rotationsNeeded = 0;

        if (ballSlot == INTAKE_SLOT) {  // Slot 0 → Slot 2
            rotationsNeeded = 2;
            lastAction = "Moving ball from INTAKE → LAUNCH (2 spins)";
        } else if (ballSlot == LAST_SLOT) {  // Slot 1 → Slot 2
            rotationsNeeded = 1;
            lastAction = "Moving ball from MIDDLE → LAUNCH (1 spin)";
        } else if (ballSlot == LAUNCH_SLOT) {  // Already at Slot 2
            rotationsNeeded = 0;
            lastAction = "Ball already at LAUNCH - firing now!";
        }

        // Rotate to bring ball to launch position
        for (int i = 0; i < rotationsNeeded; i++) {
            rotateOneSlot();
            sleep(200);  // Brief pause between rotations
        }

        // Launch the ball
        sleep(300);  // Brief pause before firing
        lastAction = "🚀 LAUNCHING " + neededColor.toUpperCase() + "!";
        telemetry.addData("STATUS", lastAction);
        telemetry.update();

        kickBall();

        // Clear the launch slot
        ballColors.put(LAUNCH_SLOT, "none");

        // Move to next needed ball
        indexOFBALLNEEDEDTOBELAUNCHED = (indexOFBALLNEEDEDTOBELAUNCHED + 1) % need_colors.length;

        lastAction = "✓ Launched! Next: " + need_colors[indexOFBALLNEEDEDTOBELAUNCHED].toUpperCase();
    }

    /**
     * Simplified, real-time telemetry
     */
    private void displayTelemetry(float hue) {

        telemetry.clear();

        // === HEADER ===
        telemetry.addLine("╔═══════════════════════════════════╗");
        telemetry.addLine("║   SPINDEXER CONTROL SYSTEM       ║");
        telemetry.addLine("╚═══════════════════════════════════╝");
        telemetry.addLine();

        // === SERVO POSITION ===
        telemetry.addData("🔧 SERVO", "%.2f° / %.0f°", targetDegrees, MAX_DEGREES);
        telemetry.addLine();

        // === SLOT CONTENTS (FIXED POSITIONS) ===
        telemetry.addLine("┌─ FIXED SLOT POSITIONS ──────┐");

        String slot0Color = ballColors.get(INTAKE_SLOT);
        String slot0Display = slot0Color.equals("none") ? "EMPTY" : "●" + slot0Color.toUpperCase();
        telemetry.addData("│ Slot 0 (INTAKE)", slot0Display);

        String slot1Color = ballColors.get(LAST_SLOT);
        String slot1Display = slot1Color.equals("none") ? "EMPTY" : "●" + slot1Color.toUpperCase();
        telemetry.addData("│ Slot 1 (MIDDLE)", slot1Display);

        String slot2Color = ballColors.get(LAUNCH_SLOT);
        String slot2Display = slot2Color.equals("none") ? "EMPTY" : "●" + slot2Color.toUpperCase();
        telemetry.addData("│ Slot 2 (LAUNCH)", slot2Display + " ◄◄ FIRE HERE");

        telemetry.addLine("└──────────────────────────────┘");
        telemetry.addLine();

        // === LAUNCH SEQUENCE ===
        telemetry.addLine("┌─ LAUNCH SEQUENCE ────────────┐");
        telemetry.addData("│ NEEDED BALL", ">>> " + need_colors[indexOFBALLNEEDEDTOBELAUNCHED].toUpperCase() + " <<<");
        telemetry.addData("│ Sequence", formatLaunchSequence());

        int neededSlot = findSlotWithColor(need_colors[indexOFBALLNEEDEDTOBELAUNCHED]);
        if (neededSlot == -1) {
            telemetry.addData("│ Status", "⚠ NOT LOADED");
        } else if (neededSlot == LAUNCH_SLOT) {
            telemetry.addData("│ Status", "✓ READY! Press RT");
        } else {
            int moves = (neededSlot == INTAKE_SLOT) ? 2 : 1;
            telemetry.addData("│ Status", "%d spin(s) needed", moves);
        }
        telemetry.addLine("└──────────────────────────────┘");
        telemetry.addLine();

        // === SENSORS & SYSTEMS ===
        String detected = "—";
        if (hue > 160 && hue < 350) {
            detected = "PURPLE ●";
        } else if (hue >= 100 && hue <= 160) {
            detected = "GREEN ●";
        }
        telemetry.addData("🎨 COLOR", detected + " (%.0f)", hue);
        telemetry.addData("🔄 INTAKE", intakeRunning ? "ON ✓" : "OFF ✗");
        telemetry.addData("⚙️ WHEEL", Math.abs(kickerWheel.getPower()) > 0.1 ? "SPINNING" : "STOPPED");
        telemetry.addLine();

        // === CONTROLS ===
        telemetry.addLine("┌─ CONTROLS ───────────────────┐");
        telemetry.addData("│ [A]", gamepad2.a ? "▶ ROTATING" : "Rotate 1 slot");
        telemetry.addData("│ [B]", gamepad2.b ? "▶ KICKING" : "Manual kick");
        telemetry.addData("│ [X]", gamepad2.x ? "▶ TOGGLING" : "Toggle intake");
        telemetry.addData("│ [Y]", "Reset position");
        telemetry.addData("│ [RT]", gamepad2.right_trigger > 0.5 ? "▶ LAUNCHING" : "🔥 AUTO LAUNCH");
        telemetry.addLine("└──────────────────────────────┘");
        telemetry.addLine();

        // === LAST ACTION ===
        telemetry.addData("📋 ACTION", lastAction);

        if (targetDegrees > MAX_DEGREES - 100) {
            telemetry.addLine();
            telemetry.addLine("⚠ NEAR MAX - Press Y to reset");
        }
    }

    /**
     * Format the launch sequence with progress indicator
     */
    private String formatLaunchSequence() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < need_colors.length; i++) {
            if (i == indexOFBALLNEEDEDTOBELAUNCHED) {
                sb.append("[").append(need_colors[i].toUpperCase()).append("]");
            } else if (i < indexOFBALLNEEDEDTOBELAUNCHED) {
                sb.append("✓");
            } else {
                sb.append(need_colors[i]);
            }
            if (i < need_colors.length - 1) {
                sb.append("→");
            }
        }
        return sb.toString();
    }

    /**
     * Rotate the spindexer by one slot (55.25°) clockwise
     * This moves balls: Slot 0→1, Slot 1→2, Slot 2→0
     */
    private void rotateOneSlot() {
        double oldTarget = targetDegrees;

        // Save current ball positions before rotation
        String ball0 = ballColors.get(INTAKE_SLOT);
        String ball1 = ballColors.get(LAST_SLOT);
        String ball2 = ballColors.get(LAUNCH_SLOT);

        // Move servo BACKWARD by STEP_DEGREES (clockwise direction)
        targetDegrees = targetDegrees - STEP_DEGREES;

        // Handle wraparound
        if (targetDegrees < 0) {
            targetDegrees += MAX_DEGREES;
        }

        // Command the servo
        indexServo.setPosition(degreesToPosition(targetDegrees));

        // Update ball positions after rotation (balls move clockwise through slots)
        ballColors.put(INTAKE_SLOT, ball2);   // Ball from Launch → Intake
        ballColors.put(LAST_SLOT, ball0);   // Ball from Intake → Middle
        ballColors.put(LAUNCH_SLOT, ball1);   // Ball from Middle → Launch

        lastAction = String.format("Rotated: %.2f°→%.2f° | Balls moved", oldTarget, targetDegrees);

        // Wait for servo to complete movement
        sleep(SERVO_MOVE_TIME_MS);
    }

    /**
     * Reset servo to initial position
     */
    private void resetToInitial() {
        targetDegrees = INITIAL_SERVO_POSITION;
        indexServo.setPosition(degreesToPosition(targetDegrees));

        lastAction = "Reset complete - Servo at " + INITIAL_SERVO_POSITION + "°";

        sleep(1500);
    }

    /**
     * Find which slot contains the specified color
     */
    private int findSlotWithColor(String color) {
        for (Map.Entry<Integer, String> entry : ballColors.entrySet()) {
            if (entry.getValue().equals(color)) {
                return entry.getKey();
            }
        }
        return -1;
    }

    /**
     * Kick ball with kicker servo AND spin the kicker wheel counterclockwise
     */
    private void kickBall() {
        double lockPosition = indexServo.getPosition();

        // Lock index servo
        indexServo.setPosition(lockPosition);

        // Fire kicker servo

        // Start kicker wheel
        kickerWheel.setPower(-1*KICKER_WHEEL_POWER);

        sleep(300);



        // Stop kicker wheel
        kickerWheel.setPower(KICKER_WHEEL_POWER);

        sleep(350
        );

        kickerWheel.setPower(0.0);

        // Re-lock index servo
        indexServo.setPosition(lockPosition);
    }

    private static double degreesToPosition(double degrees) {
        return Range.clip(degrees / MAX_DEGREES, 0.0, 1.0);
    }

    private static double positionToDegrees(double pos) {
        return Range.clip(pos, 0.0, 1.0) * MAX_DEGREES;
    }
}
package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class Spindexer {
    private LinearOpMode opMode;
    private Telemetry telemetry;

    // Hardware
    private NormalizedColorSensor intakeColorSensor;
    private Servo indexServo;
    private CRServo intakeServo;
    private Servo kickerServo;

    // State
    private int xPressCount = 0;  // counts how many balls have been shot
    private Map<Integer, String> indexColors = new HashMap<>();
    private String[] need_colors = {"purple", "purple", "green"};
    private int flag = 0;
    private int currentDivision = 0;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLeftBumper = false;
    private double targetDegrees = 0.0;
    private boolean intakeOn = false;
    private boolean rapidFireMode = true;  // Start in rapid fire mode
    private boolean allBallsIntaked = false;  // Track if all three balls are intaked

    // Constants
    public static final double MAX_DEGREES = 720.0;
    public static final double DIVISION_DEGREES = 57.0;
    public static final double SPEED_DEG_PER_STEP = 1.65;
    public static final double INITIAL_POSITION_OFFSET = 35.0;
    public static final double KICKER_RESET_POSITION = 0.89;
    public static final double KICKER_FLICK_POSITION = 0.98;
    public static final double SHOOTING_DEGREE_ADJUSTMENT = 36.0;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        indexServo = hardwareMap.get(Servo.class, "indexServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        kickerServo = hardwareMap.get(Servo.class, "kicker");

        indexColors.put(0, "none");
        indexColors.put(1, "none");
        indexColors.put(2, "none");

        // Initialize exactly as you needed earlier:
        targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
        indexServo.setPosition(posFromDeg(targetDegrees));

        telemetry.addLine("Ready. Press A to move 60°.");
        addTelemetry();
        telemetry.update();
        kickerServo.setPosition(KICKER_RESET_POSITION);
    }

    public void update(boolean gamepadA, boolean gamepadB, boolean gamepadX, boolean gamepadY, boolean gamepadLeftBumper) {
        // ---------------- MODE TOGGLE (LEFT BUMPER) -----------------
        if (gamepadLeftBumper && !prevLeftBumper) {
            rapidFireMode = !rapidFireMode;
            telemetry.addLine("Mode switched to: " + (rapidFireMode ? "RAPID FIRE" : "INDEXING"));
        }
        prevLeftBumper = gamepadLeftBumper;

        // ---------------- COLOR DETECTION -----------------
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        float hue = JavaUtil.colorToHue(colors.toColor());
        telemetry.addData("Hue", hue);
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
            
            // Check if all three balls are intaked
            int ballCount = 0;
            for (int i = 0; i < 3; i++) {
                if (!indexColors.get(i).equals("none")) {
                    ballCount++;
                }
            }
            allBallsIntaked = (ballCount >= 3);
            
            if (rapidFireMode) {
                // Rapid fire mode: go to closest ball (next division)
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                rotateOneDivision();
            } else {
                // Indexing mode: original behavior
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                rotateOneDivision();
            }
        }

        // ---------------- BUTTON Y -------------------------
        if (gamepadY && !prevY) {
            if (intakeOn) {
                intakeOn = false;
                intakeServo.setPower(0);
            } else {
                intakeOn = true;
                intakeServo.setPower(1.0);
            }
        }
        prevY = gamepadY;

        // ---------------- BUTTON A -------------------------
        if (gamepadA && !prevA) {
            targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
            smoothMoveTo(targetDegrees);
            currentDivision = (currentDivision + 1) % 3;
        }
        prevA = gamepadA;

        // ---------------- BUTTON B -------------------------
        if (gamepadB && !prevB) {
            targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
            smoothMoveTo(MAX_DEGREES - INITIAL_POSITION_OFFSET);
            currentDivision = 0;
        }
        prevB = gamepadB;

        // ---------------- BUTTON X -------------------------
        if (gamepadX && !prevX) {
            if (rapidFireMode) {
                // Rapid fire mode shooting sequence
                shootBallRapidFire();
            } else {
                // Indexing mode shooting sequence (original)
                shootBall();
            }
        }
        prevX = gamepadX;

        addTelemetry();
    }

    public void shootBall() {
        // Find the division of the next needed color
        int goalDiv = findDivisionWithColor(need_colors[(flag) % need_colors.length]); // add degree change
        if (goalDiv > 2) {
            goalDiv -= 3;
        }
        moveToDivision(goalDiv);
        rotateOneDivision();
        targetDegrees -= SHOOTING_DEGREE_ADJUSTMENT;
        smoothMoveTo(targetDegrees);
        indexColors.put(flag, "none");

        try {
            // Pause execution for 1.3 seconds (1500 milliseconds)
            Thread.sleep(1300);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            e.printStackTrace();
        }

        kickerServo.setPosition(KICKER_FLICK_POSITION); //Flick up kicker servo
        try {
            // Pause execution for 1 seconds (1000 milliseconds)
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            e.printStackTrace();
        }
        kickerServo.setPosition(KICKER_RESET_POSITION);

        try {
            // Pause execution for 1 seconds (1000 milliseconds)
            Thread.sleep(1300);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            e.printStackTrace();
        }
        targetDegrees += SHOOTING_DEGREE_ADJUSTMENT;
        smoothMoveTo(targetDegrees);

        telemetry.addLine("Ball shot!");
        // "Shoot" the ball
        xPressCount++;
        flag++; // advance to next needed color

        // After 3 shots, reset everything
        if (xPressCount >= 3) {
            //STOP SHOOTER - handled by Flywheel
            // Reset HashMap
            indexColors.put(0, "none");
            indexColors.put(1, "none");
            indexColors.put(2, "none");

            // Reset servo
            targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
            smoothMoveTo(targetDegrees);

            // Reset counters
            currentDivision = 0;
            xPressCount = 0;
            flag = 0;
            allBallsIntaked = false;
        }
    }

    public void shootBallRapidFire() {
        // Rapid fire mode shooting sequence
        // First press: just shoot (faster kicker sequence)
        // Second press: move one division and shoot
        // Third press: move one division, shoot, then reset
        
        if (xPressCount == 0) {
            // First shot: if all balls are intaked, move by SHOOTING_DEGREE_ADJUSTMENT before shooting
            if (allBallsIntaked) {
                targetDegrees = clipDeg(targetDegrees - SHOOTING_DEGREE_ADJUSTMENT);
                smoothMoveTo(targetDegrees);
            }
            performKickerSequence();
        } else if (xPressCount == 1) {
            // Second shot: move one division and shoot
            rotateOneDivision();
            performKickerSequence();
        } else if (xPressCount == 2) {
            // Third shot: move one division, shoot, then reset
            rotateOneDivision();
            performKickerSequence();
            
            // Check if all balls were intaked before resetting
            boolean wereAllBallsIntaked = allBallsIntaked;
            
            // Reset after third shot
            indexColors.put(0, "none");
            indexColors.put(1, "none");
            indexColors.put(2, "none");
            
            targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
            smoothMoveTo(targetDegrees);
            
            // Only after three balls are intaked, subtract SHOOTING_DEGREE_ADJUSTMENT
            if (wereAllBallsIntaked) {
                targetDegrees = clipDeg(targetDegrees - SHOOTING_DEGREE_ADJUSTMENT);
                smoothMoveTo(targetDegrees);
            }
            
            currentDivision = 0;
            xPressCount = -1;  // Will be incremented to 0 below
            flag = 0;
            allBallsIntaked = false;
        }
        
        xPressCount++;
        telemetry.addLine("Ball shot! (Rapid Fire)");
    }
    
    private void performKickerSequence() {
        // Faster kicker sequence for rapid fire mode
        try {
            Thread.sleep(1000);  // Reduced from 1300
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        kickerServo.setPosition(KICKER_FLICK_POSITION);
        try {
            Thread.sleep(1200);  // Reduced from 1500
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        kickerServo.setPosition(KICKER_RESET_POSITION);
        
        try {
            Thread.sleep(1000);  // Reduced from 1300
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean shouldStopFlywheel() {
        return xPressCount >= 3;
    }

    public boolean isPrevX() {
        return prevX;
    }

    // Telemetry helper
    public void addTelemetry() {
        telemetry.addData("Mode", rapidFireMode ? "RAPID FIRE" : "INDEXING");
        telemetry.addData("Target°", "%.1f", targetDegrees);
        telemetry.addData("Servo Pos", "%.3f", indexServo.getPosition());
        telemetry.addData("Division", currentDivision);

        telemetry.addData("Div0", indexColors.get(0));
        telemetry.addData("Div1", indexColors.get(1));
        telemetry.addData("Div2", indexColors.get(2));
        telemetry.addData("Need Next", need_colors[flag]);
        telemetry.addData("All Balls Intaked", allBallsIntaked);

        telemetry.addLine("A = Move 1 division");
        telemetry.addLine("B = Reset");
        telemetry.addLine("Left Bumper = Toggle Mode");
    }

    // ---------------- SMOOTH SERVO MOVEMENT (FIXED) -------------------
    public void smoothMoveTo(double newTargetDeg) {
        // ❗️THIS FIXES THE ISSUE — start from actual servo output
        double startPos = indexServo.getPosition();  // 0.0–1.0
        double startDeg = startPos * MAX_DEGREES;

        double distance = newTargetDeg - startDeg;
        int steps = (int) Math.ceil(Math.abs(distance) / SPEED_DEG_PER_STEP);

        for (int i = 1; i <= steps; i++) {
            double interpDeg = startDeg + distance * (i / (double) steps);
            indexServo.setPosition(posFromDeg(interpDeg));
            opMode.sleep(10);
        }

        indexServo.setPosition(posFromDeg(newTargetDeg));
    }

    // ---------------- DIVISION ROTATION ---------------------
    public void rotateOneDivision() {
        currentDivision = (currentDivision + 1) % 3;
        targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
        smoothMoveTo(targetDegrees);
    }

    public void moveToDivision(int targetDiv) {
        int steps = (targetDiv - currentDivision + 3) % 3;
        for (int i = 0; i < steps; i++) {
            rotateOneDivision();
            opMode.sleep(150);
        }
    }

    public int findDivisionWithColor(String c) {
        for (int i = 0; i < 3; i++) {
            if (indexColors.get(i).equals(c)) return i;
        }
        return currentDivision;
    }

    // ---------------- DEG <-> POS HELPERS ---------------------
    public static double posFromDeg(double degrees) {
        return Range.clip(degrees / MAX_DEGREES, 0.0, 1.0);
    }

    public static double clipDeg(double d) {
        return Range.clip(d, 0.0, MAX_DEGREES);
    }

    // ---------------- SIMPLE POSITION CONTROL (for testing) ---------------------
    public double getTargetDegrees() {
        return targetDegrees;
    }

    public double getServoPosition() {
        return indexServo != null ? indexServo.getPosition() : 0.0;
    }

    public void setPositionDirect(double degrees) {
        targetDegrees = clipDeg(degrees);
        if (indexServo != null) {
            indexServo.setPosition(posFromDeg(targetDegrees));
        }
    }

    public double incrementPosition(double stepDegrees) {
        targetDegrees = clipDeg(targetDegrees + stepDegrees);
        if (indexServo != null) {
            indexServo.setPosition(posFromDeg(targetDegrees));
        }
        return targetDegrees;
    }
    
    /**
     * Get the current number of balls in the spindexer
     * @return Number of balls (0-3)
     */
    public int getBallCount() {
        int ballCount = 0;
        for (int i = 0; i < 3; i++) {
            if (!indexColors.get(i).equals("none")) {
                ballCount++;
            }
        }
        return ballCount;
    }
    
    /**
     * Check if all 3 balls are intaked
     * @return true if 3 balls are present
     */
    public boolean areAllBallsIntaked() {
        return allBallsIntaked;
    }
}


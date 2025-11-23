package OpModes.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name="Turret PID Align (Lock + Correct Direction)", group="Main")
public class TurretAutoAlign extends OpMode {

    private Limelight3A limelight;
    private Servo turretServo;

    // PID constants
    private double kP = 0.00035;
    private double kI = 0.0;
    private double kD = 0.00012;

    private double integral = 0;
    private double lastError = 0;

    // Servo config
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double TARGET_TX = 11.5;  // ideal tx
    private static final double IDEAL_RANGE = 0.05; // tight tolerance
    private static boolean CHECK = false;
    private double servoPos = 0.5;
    private boolean aligned = false;
    private boolean appliedLeadOffset = false;
    private boolean lastFrameLocked = false;

    // Direction memory
    private int lastDirection = 1; // 1 = right, -1 = left

    // Scanning speed and behavior
    private double scanSpeed = 0.0008;
    private double searchRange = 0.05; // how far to scan around last seen position
    private int framesSinceSeen = 0;   // expands search zone over time if lost
    private double lockedServoPos = 0.5;

    // Max incremental adjustment
    private static final double MAX_OUTPUT = 0.0005;

    // Servo direction multiplier
    private double servoDirection = 1.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }

        turretServo.setPosition(servoPos);
        telemetry.addLine("✅ Turret PID Align Initialized");
    }

    @Override
    public void loop() {
        if (limelight == null || !limelight.isConnected()) {
            telemetry.addLine("❌ Limelight not connected");
            telemetry.update();
            return;
        }

        LLResult result = limelight.getLatestResult();
        double tx = 0;
        boolean tag24Detected = false;

        // Detect only Tag 24
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == 24) {
                    tx = fr.getTargetXDegrees();
                    tag24Detected = true;
                    break;
                }
            }
        }

        if (tag24Detected) {
            framesSinceSeen = 0;
            double error = tx - TARGET_TX;

            // ---------------- LOCK BEHAVIOR ----------------
            if (Math.abs(error) <= IDEAL_RANGE || CHECK) {
                aligned = true;
                servoPos = servoPos; // hold current position
                turretServo.setPosition(servoPos);

                integral = 0;
                lastError = 0;
                lastFrameLocked = true;

                telemetry.addLine("🎯 Aligned! Holding position");
                telemetry.addData("tx", tx);
                telemetry.addData("Error", error);
                telemetry.addData("Servo Pos", servoPos);

            } else {
                // ---------------- PID ACTIVE ----------------
                aligned = false;
                appliedLeadOffset = false;

                integral += error;
                double derivative = error - lastError;
                double rawOutput = (kP * error) + (kI * integral) + (kD * derivative);
                double output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, rawOutput));


                // Move servo toward reducing error
                double step = Math.signum(output) * Math.min(MAX_OUTPUT, Math.abs(output)); // incremental step
                servoPos += servoDirection * step;
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(servoPos);
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                turretServo.setPosition(servoPos);




                lastDirection = (servoDirection * output) >= 0 ? 1 : -1;
                lastError = error;
                lastFrameLocked = true;

                telemetry.addLine("⚡ PID Active: Correcting Position");
                telemetry.addData("tx", tx);
                telemetry.addData("Error", error);
                telemetry.addData("Servo Pos", servoPos);
                servoPos = servoPos + (error * -0.01);
                turretServo.setPosition(servoPos);
                CHECK = true;
            }

            lockedServoPos = servoPos;

        } else {
            // ---------------- TAG LOST ----------------
            framesSinceSeen++;
            integral = 0;
            aligned = false;
            appliedLeadOffset = false;

            // Reverse search direction if just lost
            if (lastFrameLocked) {
                lastDirection *= -1;
                lockedServoPos = servoPos; // start scanning from lost position
            }

            // Smart scanning within dynamic range
            double dynamicRange = Math.min(0.3, searchRange + framesSinceSeen * 0.005);
            double nextPos = servoPos + lastDirection * scanSpeed;

            if (nextPos > Math.min(SERVO_MAX, lockedServoPos + dynamicRange)) {
                lastDirection = -1;
                nextPos = Math.min(SERVO_MAX, lockedServoPos + dynamicRange);
            } else if (nextPos < Math.max(SERVO_MIN, lockedServoPos - dynamicRange)) {
                lastDirection = 1;
                nextPos = Math.max(SERVO_MIN, lockedServoPos - dynamicRange);
            }

            servoPos = nextPos;
            turretServo.setPosition(servoPos);

            telemetry.addLine("🔍 No Tag Detected — Smart Scanning");
            telemetry.addData("Servo Pos", servoPos);
            telemetry.addData("Search Range", dynamicRange);
            telemetry.addData("Scan Direction", lastDirection == 1 ? "Right" : "Left");
            lastFrameLocked = false;
        }

        telemetry.update();
    }
}

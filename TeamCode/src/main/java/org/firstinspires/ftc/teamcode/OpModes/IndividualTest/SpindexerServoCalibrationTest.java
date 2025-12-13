package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

@TeleOp(name = "Test: Spindexer Servo Calibration", group = "Test")
public class SpindexerServoCalibrationTest extends LinearOpMode {
    
    private Servo indexer;
    private double initialPosition;
    private double currentPosition;
    private double finalPosition;
    private boolean measurementComplete = false;
    private double servoPositionsPerDegree;
    private double degreesPerServoPosition;
    
    // Small increment for fine control
    private static final double SERVO_INCREMENT = 0.001; // Adjustable
    
    // Button state tracking for edge detection
    private boolean prevX = false;
    private boolean prevB = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        indexer = hardwareMap.get(Servo.class, HardwareConfig.INDEX_SERVO);
        
        // Record initial position
        initialPosition = 0;//indexer.getPosition();
        currentPosition = initialPosition;
        finalPosition = 0.0;
        measurementComplete = false;
        
        // Set initial servo position
        indexer.setPosition(currentPosition);
        
        telemetry.addLine("=== Spindexer Servo Calibration Test ===");
        telemetry.addLine("Initial servo position recorded");
        telemetry.addLine("");
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Use D-Pad Up/Down to adjust servo position");
        telemetry.addLine("2. Manually rotate spindexer 360 degrees");
        telemetry.addLine("3. Press X to record final position");
        telemetry.addLine("4. Press B to reset and start new measurement");
        telemetry.addLine("");
        addTelemetry();
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Read gamepad inputs
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;
            
            // D-Pad Up: Increment servo position
            if (dpadUp && !prevDpadUp) {
                currentPosition = Range.clip(currentPosition + SERVO_INCREMENT, 0.0, 1.0);
                indexer.setPosition(currentPosition);
            }
            prevDpadUp = dpadUp;
            
            // D-Pad Down: Decrement servo position
            if (dpadDown && !prevDpadDown) {
                currentPosition = Range.clip(currentPosition - SERVO_INCREMENT, 0.0, 1.0);
                indexer.setPosition(currentPosition);
            }
            prevDpadDown = dpadDown;
            
            // X: Record final position and calculate
            if (x && !prevX && !measurementComplete) {
                finalPosition = currentPosition;
                double servoPositionChange = finalPosition - initialPosition;
                
                if (Math.abs(servoPositionChange) > 0.0001) { // Avoid division by zero
                    servoPositionsPerDegree = servoPositionChange / 360.0;
                    degreesPerServoPosition = 360.0 / servoPositionChange;
                    measurementComplete = true;
                    telemetry.addLine("Measurement complete! Calculations updated.");
                } else {
                    telemetry.addLine("ERROR: Servo position change too small. Please adjust more.");
                }
            }
            prevX = x;
            
            // B: Reset and start new measurement
            if (b && !prevB) {
                initialPosition = currentPosition;
                finalPosition = 0.0;
                measurementComplete = false;
                servoPositionsPerDegree = 0.0;
                degreesPerServoPosition = 0.0;
                telemetry.addLine("Reset! New measurement started from current position.");
            }
            prevB = b;
            
            // Update telemetry
            addTelemetry();
            telemetry.update();
            
            idle();
        }
    }
    
    private void addTelemetry() {
        telemetry.addLine("=== SERVO POSITION ===");
        telemetry.addData("Current Position", "%.6f", currentPosition);
        telemetry.addData("Initial Position", "%.6f", initialPosition);
        
        double positionChange = currentPosition - initialPosition;
        telemetry.addData("Position Change", "%.6f", positionChange);
        
        telemetry.addLine("");
        telemetry.addLine("=== MEASUREMENT STATUS ===");
        telemetry.addData("Status", measurementComplete ? "COMPLETE" : "IN PROGRESS");
        
        if (measurementComplete) {
            telemetry.addLine("");
            telemetry.addLine("=== CALCULATIONS ===");
            telemetry.addData("Servo Positions per Degree", "%.8f", servoPositionsPerDegree);
            telemetry.addData("Degrees per Servo Position", "%.8f", degreesPerServoPosition);
            telemetry.addData("Servo Range for 360°", "%.6f", finalPosition - initialPosition);
            
            telemetry.addLine("");
            telemetry.addLine("=== GEAR RATIO VERIFICATION ===");
            telemetry.addLine("Expected: 96 teeth (servo) / 20 teeth (spindexer) = 4.8:1");
            telemetry.addLine("For 360° spindexer rotation:");
            telemetry.addLine("  Servo should rotate: 360° × 4.8 = 1728°");
            telemetry.addLine("  Servo position range: 1728° / 720° = 2.4 full rotations");
            telemetry.addLine("  Expected servo position change: ~2.4 (if MAX_DEGREES = 720)");
            
            // Calculate expected based on Spindexer.MAX_DEGREES = 720
            double expectedServoPositionChange = (360.0 * 4.8) / 720.0; // Assuming 720° max servo range
            telemetry.addData("Expected Position Change", "%.6f", expectedServoPositionChange);
            telemetry.addData("Actual Position Change", "%.6f", finalPosition - initialPosition);
            double difference = Math.abs((finalPosition - initialPosition) - expectedServoPositionChange);
            telemetry.addData("Difference", "%.6f", difference);
        }
        
        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("D-Pad Up: Increment servo (+" + String.format("%.4f", SERVO_INCREMENT) + ")");
        telemetry.addLine("D-Pad Down: Decrement servo (-" + String.format("%.4f", SERVO_INCREMENT) + ")");
        telemetry.addLine("X: Record final position (after 360° rotation)");
        telemetry.addLine("B: Reset measurement");
    }
}

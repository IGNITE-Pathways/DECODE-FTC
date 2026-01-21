package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;

/**
 * Quick Turret Test
 * 
 * Controls:
 * - D-Pad Up: Adjust turret 5 degrees up (increase position)
 * - D-Pad Down: Adjust turret 5 degrees down (decrease position)
 * 
 * This test allows precise manual adjustment of the turret position
 * for calibration and testing purposes.
 */
@TeleOp(name = "Test: Turret Quick", group = "Individual Test")
public class TurretQuickTest extends LinearOpMode {

    private Servo turretServo;
    
    // Servo position limits (from Turret.java)
    private static final double SERVO_MIN = 0.375;
    private static final double SERVO_MAX = 0.8;
    
    // Current turret position
    private double currentPosition = 0.5; // Start at middle position
    
    // Degree adjustment settings
    // Assuming turret has ~180 degrees of rotation across the servo range
    private static final double TOTAL_DEGREES = 180.0;
    private static final double DEGREES_PER_ADJUSTMENT = 5.0;
    
    // Calculate servo increment for 5 degrees
    // Servo range = 0.8 - 0.375 = 0.425
    // 5 degrees out of 180 = 5/180 = 0.0278 of the range
    // Servo increment = 0.425 * (5/180) ≈ 0.0118
    private static final double SERVO_RANGE = SERVO_MAX - SERVO_MIN;
    private static final double SERVO_INCREMENT = SERVO_RANGE * (DEGREES_PER_ADJUSTMENT / TOTAL_DEGREES);
    
    // Edge detection for button presses
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        // Initialize turret servo
        try {
            turretServo = hardwareMap.get(Servo.class, HardwareConfig.TURRET_SERVO);
            if (turretServo != null) {
                turretServo.setPosition(currentPosition);
                telemetry.addLine("✅ Turret Servo: OK");
            } else {
                telemetry.addLine("❌ Turret Servo: NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Error initializing turret servo: " + e.getMessage());
            turretServo = null;
        }
        
        telemetry.addLine("=== TURRET QUICK TEST ===");
        telemetry.addLine("D-Pad Up: +5 degrees");
        telemetry.addLine("D-Pad Down: -5 degrees");
        telemetry.addData("Current Position", "%.3f", currentPosition);
        telemetry.addData("Range", "%.3f - %.3f", SERVO_MIN, SERVO_MAX);
        telemetry.addData("Increment", "%.4f (%.1f degrees)", SERVO_INCREMENT, DEGREES_PER_ADJUSTMENT);
        telemetry.update();
        
        // Wait for start
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // Handle D-Pad Up (increase position)
            if (gamepad1.dpad_up && !prevDpadUp) {
                currentPosition += SERVO_INCREMENT;
                currentPosition = Math.min(SERVO_MAX, currentPosition); // Clamp to max
                
                if (turretServo != null) {
                    turretServo.setPosition(currentPosition);
                }
            }
            prevDpadUp = gamepad1.dpad_up;
            
            // Handle D-Pad Down (decrease position)
            if (gamepad1.dpad_down && !prevDpadDown) {
                currentPosition -= SERVO_INCREMENT;
                currentPosition = Math.max(SERVO_MIN, currentPosition); // Clamp to min
                
                if (turretServo != null) {
                    turretServo.setPosition(currentPosition);
                }
            }
            prevDpadDown = gamepad1.dpad_down;
            
            // Update telemetry
            telemetry.addLine("=== TURRET QUICK TEST ===");
            telemetry.addLine("D-Pad Up: +5 degrees");
            telemetry.addLine("D-Pad Down: -5 degrees");
            telemetry.addData("Current Position", "%.3f", currentPosition);
            telemetry.addData("Range", "%.3f - %.3f", SERVO_MIN, SERVO_MAX);
            telemetry.addData("Increment", "%.4f (%.1f degrees)", SERVO_INCREMENT, DEGREES_PER_ADJUSTMENT);
            
            // Calculate approximate degrees from center
            double centerPosition = (SERVO_MIN + SERVO_MAX) / 2.0;
            double positionOffset = currentPosition - centerPosition;
            double degreesFromCenter = (positionOffset / SERVO_RANGE) * TOTAL_DEGREES;
            telemetry.addData("Degrees from Center", "%.1f°", degreesFromCenter);
            
            telemetry.update();
        }
    }
}

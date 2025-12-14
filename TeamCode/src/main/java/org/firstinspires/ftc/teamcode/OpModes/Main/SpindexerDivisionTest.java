package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Spindexer;

/**
 * Test OpMode for calibrating spindexer division rotation
 * 
 * This OpMode helps you:
 * 1. See the current initialization position
 * 2. Test division rotation intervals
 * 3. Calibrate DIVISION_DEGREES constant
 * 
 * Controls:
 * - gamepad1.a: Rotate one division forward
 * - gamepad1.b: Reset to initial position
 * - gamepad1.x: Rotate one division backward (reverse)
 * 
 * Use this to:
 * - Verify initial position is correct
 * - Measure actual degrees per division
 * - Adjust DIVISION_DEGREES constant if needed
 */
@TeleOp(name = "Spindexer Division Test", group = "Test")
public class SpindexerDivisionTest extends LinearOpMode {
    
    private Robot robot;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    
    @Override
    public void runOpMode() {
        // Initialize robot
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, this);
        
        Spindexer spindexer = robot.getSpindexer();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Spindexer Division Rotation Test");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A = Rotate one division forward");
        telemetry.addLine("B = Reset to initial position");
        telemetry.addLine("X = Rotate one division backward");
        telemetry.addLine();
        telemetry.addData("Current Division", spindexer.getIndexColors().get(0)); // Will show division tracking
        telemetry.addData("Target Degrees", "%.2f", spindexer.getTargetDegrees());
        telemetry.addData("Servo Position", "%.4f", spindexer.getServoPosition());
        telemetry.update();
        
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // Update launcher (flywheel) - needed for spindexer updates
            robot.updateLauncher();
            
            // Handle button presses
            boolean gamepadA = gamepad1.a;
            boolean gamepadB = gamepad1.b;
            boolean gamepadX = gamepad1.x;
            
            // Rotate one division forward
            if (gamepadA && !prevA) {
                robot.rotateSpindexer();
                telemetry.addLine("Rotated one division forward");
            }
            
            // Reset to initial position
            if (gamepadB && !prevB) {
            //    spindexer.resetDivisions();
                telemetry.addLine("Reset to initial position");
            }
            
            // Rotate one division backward (reverse)
            if (gamepadX && !prevX) {
                // Manually rotate backward by adding DIVISION_DEGREES instead of subtracting
                double currentDeg = spindexer.getTargetDegrees();
                double newDeg = Spindexer.clipDeg(currentDeg + Spindexer.DIVISION_DEGREES);
                spindexer.setPositionDirect(newDeg);
                telemetry.addLine("Rotated one division backward");
            }
            
            prevA = gamepadA;
            prevB = gamepadB;
            prevX = gamepadX;
            
            // Update telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Current Division", "Division " + spindexer.getCurrentDivision());
            telemetry.addData("Target Degrees", "%.2f°", spindexer.getTargetDegrees());
            telemetry.addData("Servo Position (0-1)", "%.4f", spindexer.getServoPosition());
            telemetry.addData("Calculated Degrees", "%.2f°", spindexer.getServoPosition() * Spindexer.MAX_DEGREES);
            telemetry.addLine();
            telemetry.addData("DIVISION_DEGREES Constant", "%.2f°", Spindexer.DIVISION_DEGREES);
            telemetry.addData("MAX_DEGREES", "%.2f°", Spindexer.MAX_DEGREES);
            telemetry.addData("INITIAL_POSITION_OFFSET", "%.2f°", Spindexer.INITIAL_POSITION_OFFSET);
            telemetry.addLine();
            telemetry.addLine("Div 0: " + spindexer.getIndexColors().get(0));
            telemetry.addLine("Div 1: " + spindexer.getIndexColors().get(1));
            telemetry.addLine("Div 2: " + spindexer.getIndexColors().get(2));
            
            telemetry.update();
            
            sleep(20);
        }
    }
}


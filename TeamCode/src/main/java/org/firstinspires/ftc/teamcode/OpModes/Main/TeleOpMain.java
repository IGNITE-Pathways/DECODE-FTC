//NEED TO UPDATE FLYWHEEL SYNC AFTER KICKER MOVEMENT

package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

@TeleOp(name = "TeleOpMain", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    // Robot instance containing all components
    private Robot robot;
    
    // Alliance color: BLUE or RED
    protected AllianceColor allianceColor = null;
    
    // Button state tracking for edge detection
    private boolean prevRightBumper = false;
    private boolean prevRightTrigger = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean hasTriggeredThreeBalls = false;  // Track if we've already handled 3 balls case

    /**
     * Set the alliance color for this op mode
     * @param color AllianceColor.BLUE or AllianceColor.RED
     */
    protected void setAllianceColor(AllianceColor color) {
        this.allianceColor = color;
    }

    @Override
    public void runOpMode() {
        // Initialize robot (all components initialized within)
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, this, allianceColor);

        telemetry.addData("Status", "Initialized");
        if (allianceColor != null) {
            telemetry.addData("Alliance", allianceColor.name());
        }
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Update drive train
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            robot.updateDriveTrain(forward, right, rotate);

            // Read gamepad inputs with edge detection
            boolean rightBumper = gamepad1.right_bumper;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;  // Threshold for trigger press
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            
            // Handle intake start/stop controls
            if (rightBumper && !prevRightBumper) {
                // Start intake and color sensing
                robot.startIntake();
                robot.startColorSensing();
            }
            prevRightBumper = rightBumper;
            
            // Handle right trigger (edge) OR when ball count reaches 3 (only once)
            boolean shouldStopIntake = false;
            if (rightTrigger && !prevRightTrigger) {
                shouldStopIntake = true;
            } else if (robot.getCurrentBallCount() >= 3 && !hasTriggeredThreeBalls) {
                shouldStopIntake = true;
                hasTriggeredThreeBalls = true;
            }
            
            if (shouldStopIntake) {
                // Stop intake and color sensing
                robot.stopIntake();
                robot.stopColorSensing();
                
                // Set flywheel power and hood position
                robot.setHoodPosition(0.75);
                robot.startFlywheel();
                
                // Reset launch index when we have 3 balls loaded via Intake
                if (robot.getCurrentBallCount() >= 3) {
                    robot.resetLaunchIndex();
                }
            }
            prevRightTrigger = rightTrigger;
            
            // Reset flag when ball count drops below 3 (allows re-triggering)
            if (robot.getCurrentBallCount() < 3) {
                hasTriggeredThreeBalls = false;
            }
            
            // Handle spindexer controls
            if (a && !prevA) {
                robot.rotateSpindexer();
                // Shift balls between slots when rotating
                robot.shiftBallsBetweenSlots();
            }
            prevA = a;
            
            if (b && !prevB) {
                robot.kickSpoon();
                // Clear launch slot after kick
                robot.clearLaunchSlot();
            }
            prevB = b;
            
            if (x && !prevX) {
                robot.launchOne(telemetry);
            }
            prevX = x;

            if (robot.getLaunchIndex() >= 3) {
                // All three balls have been launched
                telemetry.addLine("All balls launched!");
                robot.resetSpindexer();
            }

            // Update color sensing (needs to be called every loop iteration when active)
            robot.updateSpindexerSensing();
            
            // Update intake (handles power ramping if enabled)
            robot.updateIntake();
            
            // Update launcher
            robot.updateLauncher();

            // Update parking (uses Lift component)
            robot.updateParking(gamepad1.dpad_up, gamepad1.dpad_down);
            
            // Add comprehensive telemetry
            robot.addIntakeTelemetry(telemetry);

            telemetry.update();
            idle();
        }
    }
}

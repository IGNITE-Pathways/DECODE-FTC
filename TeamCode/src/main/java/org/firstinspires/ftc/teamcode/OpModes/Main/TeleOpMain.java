//NEED TO UPDATE FLYWHEEL SYNC AFTER KICKER MOVEMENT

package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

/**
 * Base TeleOp class - DO NOT RUN DIRECTLY
 * Use TeleOpMainBlue or TeleOpMainRed instead
 */
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

    private boolean prevY = false;
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
        // Safety check: Warn if running TeleOpMain directly instead of through Blue/Red subclass
        // This shouldn't happen since @TeleOp annotation was removed, but defensive check
        String className = this.getClass().getSimpleName();
        if ("TeleOpMain".equals(className)) {
            telemetry.addLine("⚠️ WARNING: Running TeleOpMain directly!");
            telemetry.addLine("Please use TeleOpMainBlue or TeleOpMainRed instead");
            telemetry.addLine("Defaulting to BLUE alliance");
            telemetry.update();
            // Small delay to ensure message is visible
            sleep(2000);
        }
        
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
            if (robot.getSpindexer().needPut) {
                if (robot.getDistance() < 7) {
                    robot.setHoodPosition(0.78);
                    robot.setFlywheelPower(0.6);
                    robot.updateLauncher();
                }

                if (robot.getDistance() > 8.5) {
                    robot.setHoodPosition(0.75);
                    robot.setFlywheelPower(0.84);
                    robot.updateLauncher();
                }
            }

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
            boolean y = gamepad1.y;

            if (y && !prevY){
                robot.reverseIntake();
            }
            robot.updateTurret();
            
            // Detect obelisk ball sequence if not yet detected
            if (robot.needsObeliskDetection()) {
                robot.detectObeliskSequence();
            }
            
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

                robot.getSpindexer().needPut = true;
                // Set flywheel power and hood position


                //insert turret code


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

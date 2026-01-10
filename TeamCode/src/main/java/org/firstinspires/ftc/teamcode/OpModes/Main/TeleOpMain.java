package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

/**
 * Base TeleOp class - DO NOT RUN DIRECTLY
 * Use TeleOpMainBlue or TeleOpMainRed instead
 *
 * ============== DRIVER CONTROLS ==============
 *
 * DRIVING:
 *   Left Stick  = Drive (forward/back/strafe)
 *   Right Stick = Rotate
 *
 * SPEED (TOGGLE - click once ON, click again OFF):
 *   Left Bumper  = Toggle SLOW mode
 *   Right Bumper = Toggle TURBO mode
 *   (Both off = NORMAL mode)
 *
 * INTAKE & SHOOTING:
 *   Left Trigger (hold)  = INTAKE balls
 *   Right Trigger (hold) = SHOOT (automated)
 *
 * =============================================
 */
public class TeleOpMain extends LinearOpMode {
    // Robot instance
    private Robot robot;

    // Alliance color
    protected AllianceColor allianceColor = null;

    // ==================== STATE MACHINE ====================
    public enum RobotState {
        IDLE,       // Ready - nothing running
        INTAKING,   // Collecting balls
        SHOOTING    // Launching balls (automated)
    }

    private RobotState state = RobotState.IDLE;

    // Distance-based shooting (auto-adjusts hood based on target distance)
    private boolean useDistanceBasedHood = true;

    // Edge detection for toggle buttons
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    protected void setAllianceColor(AllianceColor color) {
        this.allianceColor = color;
    }

    @Override
    public void runOpMode() {
        // Safety check
        String className = this.getClass().getSimpleName();
        if ("TeleOpMain".equals(className)) {
            telemetry.addLine("WARNING: Use TeleOpMainBlue or TeleOpMainRed");
            telemetry.update();
            sleep(2000);
        }

        // Initialize
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, this, allianceColor);
        state = RobotState.IDLE;

        // Show controls during init
        telemetry.addLine("========== CONTROLS ==========");
        telemetry.addLine("");
        telemetry.addLine("LEFT STICK  = Drive");
        telemetry.addLine("RIGHT STICK = Rotate");
        telemetry.addLine("");
        telemetry.addLine("LEFT TRIGGER  = Intake");
        telemetry.addLine("RIGHT TRIGGER = Shoot");
        telemetry.addLine("");
        telemetry.addLine("LEFT BUMPER  = Slow Mode");
        telemetry.addLine("RIGHT BUMPER = Turbo Mode");
        telemetry.addLine("");
        telemetry.addLine("==============================");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ==================== DRIVING ====================
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            robot.updateDriveTrain(forward, strafe, rotate);

            // ==================== SPEED CONTROL (TOGGLE) ====================
            // Left Bumper = Toggle Slow, Right Bumper = Toggle Turbo
            if (gamepad1.left_bumper && !prevLeftBumper) {
                robot.getDriveTrain().toggleSlowMode();
            }
            if (gamepad1.right_bumper && !prevRightBumper) {
                robot.getDriveTrain().toggleTurboMode();
            }
            prevLeftBumper = gamepad1.left_bumper;
            prevRightBumper = gamepad1.right_bumper;

            // ==================== INTAKE / SHOOT ====================
            boolean intakePressed = gamepad1.left_trigger > 0.2;
            boolean shootPressed = gamepad1.right_trigger > 0.2;

            // Determine state based on trigger inputs
            if (shootPressed) {
                state = RobotState.SHOOTING;
            } else if (intakePressed) {
                state = RobotState.INTAKING;
            } else {
                state = RobotState.IDLE;
            }

            // Execute state actions
            switch (state) {
                case IDLE:
                    robot.getIntakeTransfer().stopIntake();
                    robot.getIntakeTransfer().transferDown();
                    robot.stopFlywheel();
                    break;

                case INTAKING:
                    robot.getIntakeTransfer().startIntake(gamepad1.left_trigger);
                    robot.getIntakeTransfer().transferDown();
                    robot.stopFlywheel();
                    break;

                case SHOOTING:
                    // Automated shooting sequence
                    robot.getIntakeTransfer().startIntake(ShooterConstants.INTAKE_SHOOTING_POWER);
                    robot.getIntakeTransfer().transferUp();

                    // Distance-based hood adjustment
                    if (useDistanceBasedHood) {
                        double distance = robot.getTurret().getDistance();
                        if (distance > 0) {
                            // Auto-adjust hood and power based on distance
                            robot.getLauncher().updateForDistance(distance);
                        } else {
                            // No valid distance - use defaults
                            robot.setFlywheelPower(ShooterConstants.FLYWHEEL_SHOOTING_POWER);
                            robot.setHoodPosition(ShooterConstants.HOOD_DEFAULT_POSITION);
                        }
                    } else {
                        // Manual mode - use default values
                        robot.setFlywheelPower(ShooterConstants.FLYWHEEL_SHOOTING_POWER);
                        robot.setHoodPosition(ShooterConstants.HOOD_DEFAULT_POSITION);
                    }

                    robot.startFlywheel();
                    break;
            }

            // Update launcher
            robot.updateLauncher();

            // ==================== TELEMETRY ====================
            telemetry.addLine("====== STATUS ======");
            telemetry.addData("State", state.name());
            telemetry.addData("Speed", robot.getDriveTrain().getSpeedMode().name());

            // Show shooting info when shooting
            if (state == RobotState.SHOOTING) {
                double dist = robot.getTurret().getDistance();
                telemetry.addData("Distance", "%.1f ft", dist);
                telemetry.addData("Hood", "%.2f", robot.getLauncher().getHoodPosition());
                telemetry.addData("Power", "%.0f%%", robot.getLauncher().getPower() * 100);
                telemetry.addData("Auto Hood", useDistanceBasedHood ? "ON" : "OFF");
            }
            telemetry.addLine("");

            telemetry.addLine("====== CONTROLS ======");
            telemetry.addLine("LT=Intake  RT=Shoot");
            String slowStatus = robot.getDriveTrain().isSlowModeActive() ? "[ON]" : "[off]";
            String turboStatus = robot.getDriveTrain().isTurboModeActive() ? "[ON]" : "[off]";
            telemetry.addLine("LB=Slow" + slowStatus + "  RB=Turbo" + turboStatus);
            telemetry.addLine("");

            robot.getDriveTrain().addTelemetry();

            telemetry.update();
            idle();
        }
    }
}

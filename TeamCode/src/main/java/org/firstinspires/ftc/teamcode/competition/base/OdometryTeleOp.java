package org.firstinspires.ftc.teamcode.competition.base;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

/**
 * Odometry-Based TeleOp - Equation-Driven Shooting
 *
 * Uses GoBilda Pinpoint (2 odo pods + IMU) for positioning
 * Calculates distance to goal using odometry
 * Auto-adjusts RPM and hood angle using equations based on tested ranges
 *
 * === CONTROLS ===
 * LEFT STICK:  Drive (100% speed)
 * RIGHT STICK: Rotate
 *
 * Y:  Toggle flywheel + raise ramp (instant!)
 * RT: Intake
 * LT: Eject
 *
 * === HOW IT WORKS ===
 * 1. Odometry tracks robot position on field
 * 2. Distance to goal calculated continuously
 * 3. RPM and hood angle auto-adjust using equations
 * 4. Press Y to turn on flywheel + raise ramp
 * 5. Press RT to intake/shoot
 */
@TeleOp(name = "Odometry TeleOp", group = "Competition")
public class OdometryTeleOp extends LinearOpMode {

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private GoBildaPinpointDriver pinpoint;

    // Alliance (set this for your alliance)
    protected AllianceColor alliance = AllianceColor.BLUE;  // Change to RED if needed

    // State
    private boolean flywheelOn = false;
    private boolean rampUp = false;

    // Robot position
    private double robotX = 0.0;
    private double robotY = 0.0;

    // Goal position (based on alliance)
    private double goalX = 0.0;
    private double goalY = 0.0;

    // Calculated values
    private double distanceToGoal = 0.0;
    private double targetRPM = RobotConstants.DEFAULT_TARGET_RPM;
    private double targetHood = RobotConstants.HOOD_DEFAULT_POSITION;

    // Button states
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            updateOdometry();
            calculateShotParameters();
            handleDriving();
            handleFlywheelControl();
            handleIntake();
            updateLauncher();
            updateTelemetry();
        }

        shutdown();
    }

    // ==================== INITIALIZATION ====================

    private void initializeHardware() {
        telemetry.addLine("=== ODOMETRY TELEOP ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize drivetrain
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);
        driveTrain.setInputCurve(RobotConstants.DRIVE_INPUT_CURVE);
        driveTrain.setRotationSensitivity(RobotConstants.ROTATION_SENSITIVITY);

        // Initialize launcher
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);
        launcher.setVelocityControlEnabled(RobotConstants.USE_VELOCITY_CONTROL);

        // Initialize intake/transfer
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        // Initialize Pinpoint odometry
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
            configurePinpoint();
            telemetry.addLine("✓ Pinpoint: OK");
        } catch (Exception e) {
            pinpoint = null;
            telemetry.addLine("✗ Pinpoint: NOT FOUND");
        }

        // Set goal position based on alliance
        if (alliance == AllianceColor.BLUE) {
            goalX = RobotConstants.BLUE_GOAL_X;
            goalY = RobotConstants.BLUE_GOAL_Y;
        } else {
            goalX = RobotConstants.RED_GOAL_X;
            goalY = RobotConstants.RED_GOAL_Y;
        }

        telemetry.addLine();
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Goal Position", "X: %.1f\" Y: %.1f\"", goalX, goalY);
        telemetry.addLine();
        telemetry.addLine("Ready! Press START.");
        telemetry.update();
    }

    private void configurePinpoint() {
        if (pinpoint == null) return;

        // Set odometry pod offsets
        pinpoint.setOffsets(RobotConstants.PINPOINT_STRAFE_POD_X,
                           RobotConstants.PINPOINT_FORWARD_POD_Y,
                           DistanceUnit.INCH);

        // Set encoder resolution (GoBilda 4-bar pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();
    }

    // ==================== ODOMETRY & DISTANCE ====================

    private void updateOdometry() {
        if (pinpoint == null) return;

        // Update Pinpoint
        pinpoint.update();

        // Get current position
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
    }

    private void calculateShotParameters() {
        // Calculate distance to goal
        distanceToGoal = RobotConstants.calculateDistanceToGoal(robotX, robotY, goalX, goalY);

        // Calculate RPM and hood using equations
        targetRPM = RobotConstants.calculateFlywheelRPM(distanceToGoal);
        targetHood = RobotConstants.calculateHoodPosition(distanceToGoal);

        // Apply to launcher if flywheel is on
        if (flywheelOn) {
            launcher.setTargetRPM(targetRPM);
            launcher.setHoodPosition(targetHood);
        }
    }

    // ==================== DRIVING ====================

    private void handleDriving() {
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Apply deadzone
        fwd = applyDeadzone(fwd);
        str = applyDeadzone(str);
        rot = applyDeadzone(rot);

        // Apply input curve to translation
        double magnitude = Math.sqrt(fwd * fwd + str * str);
        if (magnitude > 0.01) {
            double curvedMagnitude = Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
            double scale = curvedMagnitude / magnitude;
            fwd *= scale;
            str *= scale;
        }

        // Apply input curve to rotation
        rot = applyInputCurve(rot);
        rot *= RobotConstants.ROTATION_SENSITIVITY;

        // 100% speed always
        driveTrain.driveRaw(fwd, str, rot);
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < RobotConstants.JOYSTICK_DEAD_ZONE) {
            return 0;
        }
        double sign = Math.signum(input);
        return sign * (Math.abs(input) - RobotConstants.JOYSTICK_DEAD_ZONE) / (1.0 - RobotConstants.JOYSTICK_DEAD_ZONE);
    }

    private double applyInputCurve(double input) {
        if (input == 0) return 0;
        double sign = Math.signum(input);
        double magnitude = Math.abs(input);
        return sign * Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
    }

    // ==================== FLYWHEEL CONTROL ====================

    private void handleFlywheelControl() {
        // Y button: Toggle flywheel + ramp
        if (gamepad1.y && !lastY) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                // Turn on flywheel with calculated RPM
                launcher.setTargetRPM(targetRPM);
                launcher.setHoodPosition(targetHood);
                launcher.setSpinning(true);

                // Raise ramp immediately
                intakeTransfer.transferUp();
                rampUp = true;
            } else {
                // Turn off flywheel
                launcher.setSpinning(false);

                // Lower ramp
                intakeTransfer.transferDown();
                rampUp = false;
            }
        }
        lastY = gamepad1.y;
    }

    private void updateLauncher() {
        launcher.update();  // PIDF velocity control
    }

    // ==================== INTAKE CONTROL ====================

    private void handleIntake() {
        if (gamepad1.right_trigger > RobotConstants.TRIGGER_DEADZONE) {
            // Intake
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > RobotConstants.TRIGGER_DEADZONE) {
            // Eject
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            // Stop
            intakeTransfer.stopIntake();
        }
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry() {
        telemetry.addLine("=== ODOMETRY TELEOP ===");
        telemetry.addLine();

        // Robot position
        telemetry.addLine("--- POSITION ---");
        telemetry.addData("Robot", "X: %.1f\" Y: %.1f\"", robotX, robotY);
        telemetry.addData("Goal", "X: %.1f\" Y: %.1f\"", goalX, goalY);
        telemetry.addData("Distance", "%.2f ft", distanceToGoal);
        telemetry.addLine();

        // Shooting parameters
        telemetry.addLine("--- SHOOTING (Auto-Calculated) ---");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Hood Position", "%.3f", targetHood);

        if (flywheelOn) {
            double currentRPM = launcher.getCurrentRPM();
            double error = Math.abs(targetRPM - currentRPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Error", "%.0f RPM", error);

            if (error < 50) {
                telemetry.addLine("✅ READY TO SHOOT");
            } else {
                telemetry.addLine("⏳ Spinning up...");
            }
        } else {
            telemetry.addLine("⚪ Flywheel OFF (Press Y)");
        }
        telemetry.addLine();

        // State
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Flywheel", flywheelOn ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("Ramp", rampUp ? "⬆️ UP" : "⬇️ DOWN");
        telemetry.addLine();

        // Odometry status
        if (pinpoint != null) {
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        } else {
            telemetry.addLine("⚠️ Pinpoint NOT FOUND - Using default values");
        }
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Sticks: Drive (100% speed)");
        telemetry.addLine("Y: Toggle Flywheel + Ramp");
        telemetry.addLine("RT: Intake | LT: Eject");

        telemetry.update();
    }

    // ==================== SHUTDOWN ====================

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        intakeTransfer.transferDown();
        driveTrain.stopMotors();
    }
}

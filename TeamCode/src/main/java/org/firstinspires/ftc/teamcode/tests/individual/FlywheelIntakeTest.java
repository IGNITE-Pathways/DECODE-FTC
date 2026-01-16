package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Turret;

/**
 * Full Robot Test - Flywheel + Intake + Transfer + Driving + Turret
 *
 * CONTROLS:
 *   LEFT STICK    = Drive (forward/back/strafe)
 *   RIGHT STICK   = Rotate
 *
 *   RIGHT TRIGGER = Run intake
 *   LEFT TRIGGER  = Outtake / eject
 *
 *   RIGHT BUMPER  = Start flywheel
 *   LEFT BUMPER   = Stop flywheel
 *
 *   DPAD UP/DOWN    = Flywheel speed +/-
 *   DPAD LEFT/RIGHT = Hood angle +/-
 *
 *   B = Transfer ramp UP
 *   X = Transfer ramp DOWN
 */
@TeleOp(name = "Test: Full Robot", group = "Test")
public class FlywheelIntakeTest extends LinearOpMode {

    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private DriveTrain driveTrain;
    private Turret turret;

    // Edge detection
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    // State
    private boolean flywheelOn = false;
    private boolean transferUp = false;
    private double flywheelPower = 0.0;  // Start at 0%, use dpad to increase
    private double hoodPosition = 0.75;

    // Hood angle conversion (approximate)
    // Assuming servo range 0.5-0.9 maps to ~0-90 degrees
    private double servoToDegrees(double servo) {
        // Map 0.5-0.9 servo range to 0-90 degree range
        return (servo - 0.5) * 225.0; // 0.4 range = 90 degrees
    }

    @Override
    public void runOpMode() {
        // Initialize
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        turret = new Turret();
        turret.initialize(hardwareMap, telemetry, null);

        // Lock turret at fixed position
        turret.setPositionDirect(-0.2);

        telemetry.addLine("=== Full Robot Test ===");
        telemetry.addLine("");
        telemetry.addLine("DRIVING:");
        telemetry.addLine("  Left Stick = Drive");
        telemetry.addLine("  Right Stick = Rotate");
        telemetry.addLine("");
        telemetry.addLine("INTAKE:");
        telemetry.addLine("  RT = Intake");
        telemetry.addLine("  LT = Outtake");
        telemetry.addLine("");
        telemetry.addLine("SHOOTER:");
        telemetry.addLine("  RB/LB = Flywheel On/Off");
        telemetry.addLine("  D-Pad Up/Down = Flywheel Speed");
        telemetry.addLine("  D-Pad Left/Right = Hood Angle");
        telemetry.addLine("");
        telemetry.addLine("TRANSFER:");
        telemetry.addLine("  B = Up  |  X = Down");
        telemetry.addLine("");
        telemetry.addLine("Turret locked at 0.5");
        telemetry.addLine("");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========== DRIVING ==========
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrain.update(forward, strafe, rotate);

            // ========== INTAKE ==========
            if (gamepad1.right_trigger > 0.1) {
                intakeTransfer.startEject(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeTransfer.startIntake(gamepad1.left_trigger);
            } else {
                intakeTransfer.stopIntake();
            }

            // ========== FLYWHEEL ON/OFF ==========
            if (gamepad1.right_bumper && !prevRB) {
                flywheelOn = true;
                launcher.setSpinning(true);
            }
            if (gamepad1.left_bumper && !prevLB) {
                flywheelOn = false;
                launcher.setSpinning(false);
            }

            // Adjust flywheel power (can adjust while on or off)
            if (gamepad1.dpad_up && !prevDpadUp) {
                flywheelPower = Math.min(1.0, flywheelPower + 0.1);  // Increase by 10%
                launcher.setPower(flywheelPower);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                flywheelPower = Math.max(0.0, flywheelPower - 0.1);  // Decrease by 10%
                launcher.setPower(flywheelPower);
            }

            // ========== HOOD CONTROL ==========
            if (gamepad1.dpad_right && !prevDpadRight) {
                hoodPosition = Math.min(0.9, hoodPosition + 0.05);
                launcher.setHoodPosition(hoodPosition);
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                hoodPosition = Math.max(0.5, hoodPosition - 0.05);
                launcher.setHoodPosition(hoodPosition);
            }

            launcher.update();

            // ========== TRANSFER ==========
            if (gamepad1.b && !prevB) {
                transferUp = true;
                intakeTransfer.transferUp();
            }
            if (gamepad1.x && !prevX) {
                transferUp = false;
                intakeTransfer.transferDown();
            }

            // Update edge detection
            prevRB = gamepad1.right_bumper;
            prevLB = gamepad1.left_bumper;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;

            // ========== TELEMETRY ==========
            telemetry.clear();
            telemetry.addLine("╔═══════════════════════════════╗");
            telemetry.addLine("║    FULL ROBOT TEST STATUS    ║");
            telemetry.addLine("╚═══════════════════════════════╝");
            telemetry.addLine();

            // Shooter Status
            telemetry.addLine("┌─ SHOOTER ─────────────────┐");
            telemetry.addData("│ Flywheel", flywheelOn ? "ON ✓" : "OFF");
            telemetry.addData("│ Speed", "%.0f%%", flywheelPower * 100);
            telemetry.addData("│ Motor 1 Power", "%.3f", launcher.flyWheelMotor.getPower());
            telemetry.addData("│ Motor 2 Power", "%.3f", launcher.flyWheelMotor2.getPower());
            telemetry.addData("│ Hood Position", "%.3f", hoodPosition);
            telemetry.addData("│ Hood Angle", "%.1f°", servoToDegrees(hoodPosition));
            telemetry.addLine("└───────────────────────────┘");
            telemetry.addLine();

            // Intake/Transfer Status
            telemetry.addLine("┌─ INTAKE & TRANSFER ───────┐");
            String intakeStatus = gamepad1.right_trigger > 0.1 ? "RUNNING ►" :
                                  (gamepad1.left_trigger > 0.1 ? "EJECTING ◄" : "STOPPED");
            telemetry.addData("│ Intake", intakeStatus);
            telemetry.addData("│ Transfer", transferUp ? "UP ▲" : "DOWN ▼");
            telemetry.addLine("└───────────────────────────┘");
            telemetry.addLine();

            // Driving Status
            telemetry.addLine("┌─ DRIVING ─────────────────┐");
            telemetry.addData("│ Forward", "%.2f", forward);
            telemetry.addData("│ Strafe", "%.2f", strafe);
            telemetry.addData("│ Rotate", "%.2f", rotate);
            telemetry.addLine("└───────────────────────────┘");
            telemetry.addLine();

            // Turret Status
            telemetry.addData("Turret Position", "0.5 (LOCKED)");
            telemetry.addLine();

            // Controls
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("CONTROLS:");
            telemetry.addData("  RT/LT", "Intake/Outtake");
            telemetry.addData("  RB/LB", "Flywheel On/Off");
            telemetry.addData("  D↑/D↓", "Flywheel %.0f%%", flywheelPower * 100);
            telemetry.addData("  D←/D→", "Hood %.1f°", servoToDegrees(hoodPosition));
            telemetry.addData("  B/X", "Transfer Up/Down");
            telemetry.addLine("═══════════════════════════════");

            telemetry.update();

            idle();
        }
    }
}

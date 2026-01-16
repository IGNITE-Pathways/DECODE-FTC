package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.TeleOpConstants;

/**
 * Main TeleOp Program with Automated Controls
 *
 * DRIVER CONTROLS:
 *   LEFT STICK     = Drive (forward/back/strafe)
 *   RIGHT STICK    = Rotate
 *
 *   RIGHT TRIGGER  = Run intake
 *   LEFT TRIGGER   = Eject/outtake
 *
 *   Y BUTTON       = AUTO SHOOT (flywheel + ramp + intake)
 *   RIGHT BUMPER   = Start flywheel only
 *   LEFT BUMPER    = Stop flywheel
 *
 *   DPAD UP/DOWN   = Flywheel speed +/-
 *   DPAD LEFT/RIGHT = Hood angle +/-
 *
 *   B              = Transfer ramp UP (manual)
 *   X              = Transfer ramp DOWN (manual)
 *
 *   A              = Toggle turret auto-lock
 *   BACK           = Toggle alliance color (RED/BLUE)
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private TurretLockOptimized turret;

    // State variables
    private boolean flywheelOn = false;
    private boolean transferUp = false;
    private double flywheelPower = TeleOpConstants.FLYWHEEL_DEFAULT_POWER;
    private double hoodPosition = TeleOpConstants.HOOD_DEFAULT_POSITION;
    private boolean turretAutoLock = TeleOpConstants.TURRET_AUTO_LOCK_ENABLED;
    private AllianceColor alliance = AllianceColor.BLUE;

    // Auto shoot state machine
    private enum AutoShootState {
        IDLE,
        SPIN_UP_FLYWHEEL,
        RAISE_RAMP,
        FEEDING,
        COMPLETE
    }
    private AutoShootState autoShootState = AutoShootState.IDLE;
    private ElapsedTime autoShootTimer = new ElapsedTime();

    // Edge detection for buttons
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevA = false;
    private boolean prevBack = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    @Override
    public void runOpMode() {
        // Initialize all components
        initializeRobot();

        telemetry.addLine("=== Main TeleOp Ready ===");
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Turret Auto-Lock", turretAutoLock ? "ON" : "OFF");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========== DRIVING ==========
            updateDriving();

            // ========== INTAKE ==========
            updateIntake();

            // ========== FLYWHEEL MANUAL CONTROL ==========
            updateFlywheelManual();

            // ========== HOOD CONTROL ==========
            updateHood();

            // ========== TRANSFER RAMP MANUAL CONTROL ==========
            updateTransferManual();

            // ========== AUTO SHOOT (Y BUTTON) ==========
            updateAutoShoot();

            // ========== TURRET CONTROL ==========
            updateTurret();

            // ========== ALLIANCE TOGGLE ==========
            if (gamepad1.back && !prevBack) {
                toggleAlliance();
            }
            prevBack = gamepad1.back;

            // ========== UPDATE ALL COMPONENTS ==========
            launcher.update();

            // ========== TELEMETRY ==========
            displayTelemetry();

            idle();
        }
    }

    private void initializeRobot() {
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);
        launcher.setPower(flywheelPower);
        launcher.setHoodPosition(hoodPosition);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, alliance);
    }

    private void updateDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        driveTrain.update(forward, strafe, rotate);
    }

    private void updateIntake() {
        // Only manual control if not in auto shoot mode
        if (autoShootState == AutoShootState.IDLE) {
            if (gamepad1.right_trigger > 0.1) {
                intakeTransfer.startIntake(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeTransfer.startEject(gamepad1.left_trigger);
            } else {
                intakeTransfer.stopIntake();
            }
        }
    }

    private void updateFlywheelManual() {
        // Only allow manual control if not in auto shoot
        if (autoShootState == AutoShootState.IDLE) {
            // Right bumper = flywheel on
            if (gamepad1.right_bumper && !prevRB) {
                flywheelOn = true;
                launcher.setSpinning(true);
            }

            // Left bumper = flywheel off
            if (gamepad1.left_bumper && !prevLB) {
                flywheelOn = false;
                launcher.setSpinning(false);
            }

            // Dpad up/down = adjust flywheel power
            if (gamepad1.dpad_up && !prevDpadUp) {
                flywheelPower = Math.min(1.0, flywheelPower + TeleOpConstants.FLYWHEEL_POWER_INCREMENT);
                launcher.setPower(flywheelPower);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                flywheelPower = Math.max(0.0, flywheelPower - TeleOpConstants.FLYWHEEL_POWER_INCREMENT);
                launcher.setPower(flywheelPower);
            }
        }

        prevRB = gamepad1.right_bumper;
        prevLB = gamepad1.left_bumper;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
    }

    private void updateHood() {
        // Hood control with dpad left/right
        if (gamepad1.dpad_right && !prevDpadRight) {
            hoodPosition = Math.min(TeleOpConstants.HOOD_MAX, hoodPosition + TeleOpConstants.HOOD_INCREMENT);
            launcher.setHoodPosition(hoodPosition);
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            hoodPosition = Math.max(TeleOpConstants.HOOD_MIN, hoodPosition - TeleOpConstants.HOOD_INCREMENT);
            launcher.setHoodPosition(hoodPosition);
        }

        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
    }

    private void updateTransferManual() {
        // Only allow manual control if not in auto shoot
        if (autoShootState == AutoShootState.IDLE) {
            if (gamepad1.b && !prevB) {
                transferUp = true;
                intakeTransfer.transferUp();
            }
            if (gamepad1.x && !prevX) {
                transferUp = false;
                intakeTransfer.transferDown();
            }
        }

        prevB = gamepad1.b;
        prevX = gamepad1.x;
    }

    private void updateAutoShoot() {
        // Y button starts auto shoot sequence
        if (gamepad1.y && !prevY && autoShootState == AutoShootState.IDLE) {
            startAutoShoot();
        }
        prevY = gamepad1.y;

        // Auto shoot state machine
        switch (autoShootState) {
            case IDLE:
                // Do nothing
                break;

            case SPIN_UP_FLYWHEEL:
                if (autoShootTimer.milliseconds() >= TeleOpConstants.AUTO_SHOOT_RAMP_UP_DELAY_MS) {
                    // Move to next state
                    autoShootState = AutoShootState.RAISE_RAMP;
                    intakeTransfer.transferUp();
                    transferUp = true;
                    autoShootTimer.reset();
                }
                break;

            case RAISE_RAMP:
                // Give ramp a moment to move, then start feeding
                if (autoShootTimer.milliseconds() >= 200) {
                    autoShootState = AutoShootState.FEEDING;
                    intakeTransfer.startIntake();
                    autoShootTimer.reset();
                }
                break;

            case FEEDING:
                if (autoShootTimer.milliseconds() >= TeleOpConstants.AUTO_SHOOT_INTAKE_DELAY_MS) {
                    // Done feeding, complete sequence
                    autoShootState = AutoShootState.COMPLETE;
                    autoShootTimer.reset();
                }
                break;

            case COMPLETE:
                // Stop everything after a brief moment
                if (autoShootTimer.milliseconds() >= 200) {
                    stopAutoShoot();
                }
                break;
        }
    }

    private void startAutoShoot() {
        // Start auto shoot sequence
        autoShootState = AutoShootState.SPIN_UP_FLYWHEEL;
        autoShootTimer.reset();

        // Start flywheel at auto shoot power
        flywheelPower = TeleOpConstants.FLYWHEEL_AUTO_SHOOT_POWER;
        launcher.setPower(flywheelPower);
        launcher.setSpinning(true);
        flywheelOn = true;
    }

    private void stopAutoShoot() {
        // Return to idle state
        autoShootState = AutoShootState.IDLE;

        // Stop intake (keep flywheel running if driver wants)
        intakeTransfer.stopIntake();

        // Lower transfer ramp
        intakeTransfer.transferDown();
        transferUp = false;
    }

    private void updateTurret() {
        // A button toggles auto-lock
        if (gamepad1.a && !prevA) {
            turretAutoLock = !turretAutoLock;
            if (!turretAutoLock) {
                // If disabling auto-lock, lock turret at fixed position
                turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
            }
        }
        prevA = gamepad1.a;

        // Update turret
        if (turretAutoLock) {
            turret.update();  // Auto-lock enabled
        } else {
            // Manual lock at fixed position
            turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
        }
    }

    private void toggleAlliance() {
        if (alliance == AllianceColor.BLUE) {
            alliance = AllianceColor.RED;
        } else {
            alliance = AllianceColor.BLUE;
        }
        turret.setAlliance(alliance);
    }

    private void displayTelemetry() {
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║      MAIN TELEOP STATUS      ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();

        // Shooter Status
        telemetry.addLine("┌─ SHOOTER ─────────────────┐");
        telemetry.addData("│ Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("│ Power", "%.0f%%", flywheelPower * 100);
        telemetry.addData("│ Hood", "%.2f", hoodPosition);
        telemetry.addLine("└───────────────────────────┘");
        telemetry.addLine();

        // Transfer & Intake
        telemetry.addLine("┌─ INTAKE & TRANSFER ───────┐");
        String intakeStatus = "STOPPED";
        if (autoShootState != AutoShootState.IDLE) {
            intakeStatus = "AUTO";
        } else if (gamepad1.right_trigger > 0.1) {
            intakeStatus = "INTAKE";
        } else if (gamepad1.left_trigger > 0.1) {
            intakeStatus = "EJECT";
        }
        telemetry.addData("│ Intake", intakeStatus);
        telemetry.addData("│ Transfer", transferUp ? "UP" : "DOWN");
        telemetry.addLine("└───────────────────────────┘");
        telemetry.addLine();

        // Turret Status
        telemetry.addLine("┌─ TURRET ──────────────────┐");
        telemetry.addData("│ Mode", turretAutoLock ? "AUTO-LOCK" : "MANUAL");
        if (turretAutoLock) {
            telemetry.addData("│ Status", turret.getStateName());
            telemetry.addData("│ Locked", turret.isLocked() ? "YES" : "NO");
            telemetry.addData("│ Tag Visible", turret.isTagVisible() ? "YES" : "NO");
        } else {
            telemetry.addData("│ Position", "%.3f", TeleOpConstants.TURRET_LOCKED_POSITION);
        }
        telemetry.addLine("└───────────────────────────┘");
        telemetry.addLine();

        // Auto Shoot Status
        if (autoShootState != AutoShootState.IDLE) {
            telemetry.addLine("┌─ AUTO SHOOT ──────────────┐");
            telemetry.addData("│ State", autoShootState.name());
            telemetry.addData("│ Timer", "%.1f s", autoShootTimer.seconds());
            telemetry.addLine("└───────────────────────────┘");
            telemetry.addLine();
        }

        // Alliance & Config
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Drive Speed", driveTrain.getSpeedMode().name());

        telemetry.update();
    }
}

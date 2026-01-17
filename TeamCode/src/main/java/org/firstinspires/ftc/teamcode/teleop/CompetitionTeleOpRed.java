package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.TeleOpConstants;

import java.util.List;

/**
 * RED ALLIANCE Competition TeleOp
 * Turret tracks AprilTag ID 24 (Red Alliance)
 *
 * DRIVER CONTROLS (Gamepad 1):
 *   LEFT STICK      = Drive (forward/back/strafe)
 *   RIGHT STICK X   = Rotate
 *   RIGHT STICK Y   = Hood adjustment (up/down)
 *
 *   RIGHT TRIGGER   = Run intake
 *   LEFT TRIGGER    = Eject/outtake
 *
 *   Y BUTTON        = AUTO SHOOT (automated shooting sequence)
 *   RIGHT BUMPER    = Flywheel ON
 *   LEFT BUMPER     = Flywheel OFF
 *
 *   DPAD UP/DOWN    = Flywheel power +/- (10% increments)
 *   DPAD LEFT/RIGHT = Drive speed mode (slow/normal/turbo)
 *
 *   A BUTTON        = Toggle turret auto-lock
 *   B BUTTON        = Transfer ramp UP
 *   X BUTTON        = Transfer ramp DOWN
 *
 *   START + BACK    = EMERGENCY STOP ALL
 */
@TeleOp(name = "Competition TeleOp RED", group = "Competition")
public class CompetitionTeleOpRed extends LinearOpMode {

    // ==================== ALLIANCE ====================
    private static final AllianceColor ALLIANCE = AllianceColor.RED;

    // ==================== COMPONENTS ====================
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private TurretLockOptimized turret;

    // Bulk caching for performance
    private List<LynxModule> allHubs;

    // ==================== STATE VARIABLES ====================
    private boolean flywheelOn = false;
    private boolean transferUp = false;
    private double flywheelPower = TeleOpConstants.FLYWHEEL_DEFAULT_POWER;
    private double hoodPosition = TeleOpConstants.HOOD_DEFAULT_POSITION;
    private boolean turretAutoLock = TeleOpConstants.TURRET_AUTO_LOCK_ENABLED;

    // Emergency stop flag
    private boolean emergencyStop = false;

    // ==================== AUTO SHOOT STATE MACHINE ====================
    private enum AutoShootState {
        IDLE,
        SPIN_UP_FLYWHEEL,
        RAISE_RAMP,
        FEEDING,
        COMPLETE
    }
    private AutoShootState autoShootState = AutoShootState.IDLE;
    private ElapsedTime autoShootTimer = new ElapsedTime();

    // ==================== TIMERS & MONITORING ====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double averageLoopTime = 0;
    private int loopCount = 0;

    // ==================== BUTTON EDGE DETECTION ====================
    private boolean prevG1_RB = false;
    private boolean prevG1_LB = false;
    private boolean prevG1_A = false;
    private boolean prevG1_B = false;
    private boolean prevG1_X = false;
    private boolean prevG1_Y = false;
    private boolean prevG1_DpadUp = false;
    private boolean prevG1_DpadDown = false;
    private boolean prevG1_DpadLeft = false;
    private boolean prevG1_DpadRight = false;

    @Override
    public void runOpMode() {
        // ==================== INITIALIZATION ====================
        try {
            initializeRobot();
        } catch (Exception e) {
            telemetry.addLine("INITIALIZATION FAILED!");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            return;
        }

        // ==================== WAIT FOR START ====================
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║     RED ALLIANCE TELEOP      ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Alliance", ALLIANCE.name());
        telemetry.addData("Target AprilTag", "24 (Red)");
        telemetry.addData("Turret Auto-Lock", turretAutoLock ? "ENABLED" : "DISABLED");
        telemetry.addData("Battery", "%.1f V (%d%%)",
            driveTrain.getBatteryVoltage(), driveTrain.getBatteryPercentage());
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ==================== MAIN LOOP ====================
        while (opModeIsActive() && !emergencyStop) {
            loopTimer.reset();

            // Clear bulk cache for new reads
            clearBulkCache();

            // ========== EMERGENCY STOP CHECK ==========
            if (gamepad1.start && gamepad1.back) {
                emergencyStopAll();
                break;
            }

            // ========== UPDATE ALL SYSTEMS ==========
            updateDriving();
            updateIntake();
            updateFlywheelManual();
            updateHood();
            updateTransferManual();
            updateAutoShoot();
            updateTurret();
            updateDriveSpeed();

            // ========== UPDATE COMPONENTS ==========
            try {
                launcher.update();
            } catch (Exception e) {
                telemetry.addLine("LAUNCHER ERROR!");
                launcher.setSpinning(false);
            }

            // ========== TELEMETRY ==========
            displayTelemetry();

            // ========== UPDATE EDGE DETECTION (MUST BE AT END) ==========
            updateEdgeDetection();

            // ========== LOOP TIME MONITORING ==========
            trackLoopTime();

            idle();
        }

        // ==================== SHUTDOWN ====================
        shutdown();
    }

    // ==================== INITIALIZATION ====================

    private void initializeRobot() {
        telemetry.addLine("Initializing RED alliance robot...");
        telemetry.update();

        // Enable bulk caching for ~2.5x performance boost
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize components
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);
        launcher.setPower(flywheelPower);
        launcher.setHoodPosition(hoodPosition);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, ALLIANCE);
    }

    // ==================== BULK CACHING ====================

    private void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    // ==================== UPDATE METHODS ====================

    private void updateDriving() {
        if (emergencyStop) {
            driveTrain.stopMotors();
            return;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driveTrain.update(forward, strafe, rotate);
    }

    private void updateIntake() {
        if (emergencyStop) {
            intakeTransfer.stopIntake();
            return;
        }

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
        if (emergencyStop) {
            launcher.setSpinning(false);
            return;
        }

        // Only allow manual control if not in auto shoot
        if (autoShootState == AutoShootState.IDLE) {
            // Right bumper = flywheel on
            if (gamepad1.right_bumper && !prevG1_RB) {
                flywheelOn = true;
                launcher.setSpinning(true);
            }

            // Left bumper = flywheel off
            if (gamepad1.left_bumper && !prevG1_LB) {
                flywheelOn = false;
                launcher.setSpinning(false);
            }

            // Dpad up/down = adjust flywheel power
            if (gamepad1.dpad_up && !prevG1_DpadUp) {
                flywheelPower = Math.min(1.0, flywheelPower + TeleOpConstants.FLYWHEEL_POWER_INCREMENT);
                launcher.setPower(flywheelPower);
            }
            if (gamepad1.dpad_down && !prevG1_DpadDown) {
                flywheelPower = Math.max(0.0, flywheelPower - TeleOpConstants.FLYWHEEL_POWER_INCREMENT);
                launcher.setPower(flywheelPower);
            }
        }
    }

    private void updateHood() {
        if (emergencyStop) return;

        // Hood control with RIGHT STICK Y
        double hoodAdjust = -gamepad1.right_stick_y * 0.01; // Smooth analog control
        if (Math.abs(hoodAdjust) > 0.05) {
            hoodPosition = Math.max(TeleOpConstants.HOOD_MIN,
                            Math.min(TeleOpConstants.HOOD_MAX, hoodPosition + hoodAdjust));
            launcher.setHoodPosition(hoodPosition);
        }
    }

    private void updateTransferManual() {
        if (emergencyStop) {
            intakeTransfer.transferDown();
            return;
        }

        // Only allow manual control if not in auto shoot
        if (autoShootState == AutoShootState.IDLE) {
            if (gamepad1.b && !prevG1_B) {
                transferUp = true;
                intakeTransfer.transferUp();
            }
            if (gamepad1.x && !prevG1_X) {
                transferUp = false;
                intakeTransfer.transferDown();
            }
        }
    }

    private void updateAutoShoot() {
        if (emergencyStop) {
            if (autoShootState != AutoShootState.IDLE) {
                stopAutoShoot();
            }
            return;
        }

        // Y button starts auto shoot sequence
        if (gamepad1.y && !prevG1_Y && autoShootState == AutoShootState.IDLE) {
            startAutoShoot();
        }

        // Auto shoot state machine
        switch (autoShootState) {
            case IDLE:
                break;

            case SPIN_UP_FLYWHEEL:
                if (autoShootTimer.milliseconds() >= TeleOpConstants.AUTO_SHOOT_RAMP_UP_DELAY_MS) {
                    autoShootState = AutoShootState.RAISE_RAMP;
                    intakeTransfer.transferUp();
                    transferUp = true;
                    autoShootTimer.reset();
                }
                break;

            case RAISE_RAMP:
                if (autoShootTimer.milliseconds() >= 200) {
                    autoShootState = AutoShootState.FEEDING;
                    intakeTransfer.startIntake();
                    autoShootTimer.reset();
                }
                break;

            case FEEDING:
                if (autoShootTimer.milliseconds() >= TeleOpConstants.AUTO_SHOOT_INTAKE_DELAY_MS) {
                    autoShootState = AutoShootState.COMPLETE;
                    autoShootTimer.reset();
                }
                break;

            case COMPLETE:
                if (autoShootTimer.milliseconds() >= 200) {
                    stopAutoShoot();
                }
                break;
        }
    }

    private void startAutoShoot() {
        autoShootState = AutoShootState.SPIN_UP_FLYWHEEL;
        autoShootTimer.reset();

        flywheelPower = TeleOpConstants.FLYWHEEL_AUTO_SHOOT_POWER;
        launcher.setPower(flywheelPower);
        launcher.setSpinning(true);
        flywheelOn = true;
    }

    private void stopAutoShoot() {
        autoShootState = AutoShootState.IDLE;
        intakeTransfer.stopIntake();
        intakeTransfer.transferDown();
        transferUp = false;
    }

    private void updateTurret() {
        if (emergencyStop) return;

        // A button toggles auto-lock
        if (gamepad1.a && !prevG1_A) {
            turretAutoLock = !turretAutoLock;
            if (!turretAutoLock) {
                turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
            }
        }

        // Update turret
        try {
            if (turretAutoLock) {
                turret.update();
            } else {
                turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
            }
        } catch (Exception e) {
            turretAutoLock = false;
            turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
        }
    }

    private void updateDriveSpeed() {
        if (emergencyStop) return;

        if (gamepad1.dpad_left && !prevG1_DpadLeft) {
            if (driveTrain.getSpeedMode() == DriveTrain.SpeedMode.SLOW) {
                driveTrain.setSpeedMode(DriveTrain.SpeedMode.TURBO);
            } else if (driveTrain.getSpeedMode() == DriveTrain.SpeedMode.NORMAL) {
                driveTrain.setSpeedMode(DriveTrain.SpeedMode.SLOW);
            } else {
                driveTrain.setSpeedMode(DriveTrain.SpeedMode.NORMAL);
            }
        }

        if (gamepad1.dpad_right && !prevG1_DpadRight) {
            if (driveTrain.getSpeedMode() == DriveTrain.SpeedMode.SLOW) {
                driveTrain.setSpeedMode(DriveTrain.SpeedMode.NORMAL);
            } else if (driveTrain.getSpeedMode() == DriveTrain.SpeedMode.NORMAL) {
                driveTrain.setSpeedMode(DriveTrain.SpeedMode.TURBO);
            } else {
                driveTrain.setSpeedMode(DriveTrain.SpeedMode.SLOW);
            }
        }
    }

    // ==================== EMERGENCY STOP ====================

    private void emergencyStopAll() {
        emergencyStop = true;

        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║    EMERGENCY STOP ACTIVE!    ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.update();

        try { launcher.setSpinning(false); } catch (Exception ignored) {}
        try { intakeTransfer.stopIntake(); } catch (Exception ignored) {}
        try { intakeTransfer.transferDown(); } catch (Exception ignored) {}
        try { driveTrain.stopMotors(); } catch (Exception ignored) {}
    }

    // ==================== EDGE DETECTION ====================

    private void updateEdgeDetection() {
        prevG1_RB = gamepad1.right_bumper;
        prevG1_LB = gamepad1.left_bumper;
        prevG1_A = gamepad1.a;
        prevG1_B = gamepad1.b;
        prevG1_X = gamepad1.x;
        prevG1_Y = gamepad1.y;
        prevG1_DpadUp = gamepad1.dpad_up;
        prevG1_DpadDown = gamepad1.dpad_down;
        prevG1_DpadLeft = gamepad1.dpad_left;
        prevG1_DpadRight = gamepad1.dpad_right;
    }

    // ==================== LOOP TIME TRACKING ====================

    private void trackLoopTime() {
        double currentLoopTime = loopTimer.milliseconds();
        loopCount++;
        averageLoopTime = ((averageLoopTime * (loopCount - 1)) + currentLoopTime) / loopCount;
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry() {
        telemetry.clear();

        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║    RED ALLIANCE TELEOP       ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();

        telemetry.addData("⏱ Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("⚡ Loop Time", "%.1f ms (avg: %.1f ms)", loopTimer.milliseconds(), averageLoopTime);
        telemetry.addLine();

        double voltage = driveTrain.getBatteryVoltage();
        int batteryPct = driveTrain.getBatteryPercentage();
        String batteryIcon = batteryPct > 50 ? "🔋" : (batteryPct > 20 ? "🪫" : "⚠️");
        telemetry.addData(batteryIcon + " Battery", "%.1f V (%d%%)", voltage, batteryPct);
        if (driveTrain.isBatteryCritical()) {
            telemetry.addLine("⚠️ BATTERY CRITICAL - REPLACE SOON!");
        } else if (driveTrain.isBatteryLow()) {
            telemetry.addLine("⚠️ Battery Low");
        }
        telemetry.addLine();

        telemetry.addLine("┌─ SHOOTER ─────────────────┐");
        telemetry.addData("│ Flywheel", flywheelOn ? "✓ ON" : "OFF");
        telemetry.addData("│ Power", "%.0f%%", flywheelPower * 100);
        telemetry.addData("│ Hood", "%.2f", hoodPosition);
        telemetry.addLine("└───────────────────────────┘");
        telemetry.addLine();

        telemetry.addLine("┌─ INTAKE & TRANSFER ───────┐");
        String intakeStatus = "STOPPED";
        if (autoShootState != AutoShootState.IDLE) {
            intakeStatus = "AUTO (" + autoShootState.name() + ")";
        } else if (gamepad1.right_trigger > 0.1) {
            intakeStatus = "INTAKE ►";
        } else if (gamepad1.left_trigger > 0.1) {
            intakeStatus = "EJECT ◄";
        }
        telemetry.addData("│ Status", intakeStatus);
        telemetry.addData("│ Transfer", transferUp ? "UP ▲" : "DOWN ▼");
        telemetry.addLine("└───────────────────────────┘");
        telemetry.addLine();

        telemetry.addLine("┌─ TURRET ──────────────────┐");
        telemetry.addData("│ Mode", turretAutoLock ? "AUTO-LOCK" : "MANUAL");
        if (turretAutoLock) {
            telemetry.addData("│ State", turret.getStateName());
            telemetry.addData("│ Locked", turret.isLocked() ? "✓ YES" : "NO");
            telemetry.addData("│ Tag", turret.isTagVisible() ? "✓ VISIBLE (ID 24)" : "NOT VISIBLE");
        } else {
            telemetry.addData("│ Position", "%.3f (LOCKED)", TeleOpConstants.TURRET_LOCKED_POSITION);
        }
        telemetry.addLine("└───────────────────────────┘");
        telemetry.addLine();

        telemetry.addData("🏁 Alliance", ALLIANCE.name() + " (Tag 24)");
        telemetry.addData("🚗 Drive Speed", driveTrain.getSpeedMode().name() +
            " (" + (int)(driveTrain.getSpeedMode().multiplier * 100) + "%)");

        telemetry.update();
    }

    // ==================== SHUTDOWN ====================

    private void shutdown() {
        try {
            launcher.setSpinning(false);
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
            driveTrain.stopMotors();
        } catch (Exception ignored) {}

        telemetry.addLine("TeleOp ended. All systems stopped.");
        telemetry.update();
    }
}

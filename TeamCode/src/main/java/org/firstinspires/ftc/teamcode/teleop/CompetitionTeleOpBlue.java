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
 * BLUE ALLIANCE Competition TeleOp - Turret tracks AprilTag ID 20
 *
 * GP1: Drive (sticks)
 * GP2: Triggers = intake/eject, D-Pad = presets, Y (hold) = auto shoot
 * Turret auto-tracks continuously
 */
@TeleOp(name = "Competition TeleOp BLUE", group = "Competition")
public class CompetitionTeleOpBlue extends LinearOpMode {

    private static final AllianceColor ALLIANCE = AllianceColor.BLUE;

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private TurretLockOptimized turret;

    // State
    private boolean flywheelOn = false;
    private double flywheelPower = 0.55;
    private double hoodPosition = 0.35;
    private String selectedPreset = "2 FT";
    private boolean transferUp = false;
    private boolean shooting = false;
    private boolean turretTracking = false;

    // Button states for toggle
    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    // Speed mode: 0 = normal (70%), 1 = slow (40%), 2 = fast (100%)
    private int speedMode = 0;


    // Auto shoot timer
    private ElapsedTime shootTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Init
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, ALLIANCE);

        telemetry.addLine("BLUE TeleOp Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // === DRIVING (GP1) ===
            double fwd = -gamepad1.left_stick_y;
            double str = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            // Speed mode toggle: LB = slow, RB = fast
            if (gamepad1.left_bumper && !lastLB) {
                speedMode = (speedMode == 1) ? 0 : 1;  // Toggle slow
            }
            if (gamepad1.right_bumper && !lastRB) {
                speedMode = (speedMode == 2) ? 0 : 2;  // Toggle fast
            }
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            // Apply speed multiplier
            double speedMult = (speedMode == 1) ? 0.4 : (speedMode == 2) ? 1.0 : 0.7;
            driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);

            // === TURRET - A toggles auto-track ===
            if (gamepad1.a && !lastA) {
                turretTracking = !turretTracking;
            }
            lastA = gamepad1.a;

            if (turretTracking) {
                turret.update();  // Live tracking
            } else {
                turret.setPositionDirect(TeleOpConstants.TURRET_LOCKED_POSITION);
            }

            // === INTAKE (GP1 triggers) - only when not shooting ===
            if (!shooting) {
                if (gamepad1.right_trigger > 0.1) {
                    intakeTransfer.startIntake(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0.1) {
                    intakeTransfer.startEject(gamepad1.left_trigger);
                } else {
                    intakeTransfer.stopIntake();
                }
            }

            // === PRESETS (GP2 D-Pad) - Apply instantly when pressed ===
            if (gamepad2.dpad_up) {
                selectedPreset = "2 FT";
                flywheelPower = TeleOpConstants.PRESET_2FT_POWER;
                hoodPosition = TeleOpConstants.PRESET_2FT_HOOD;
                launcher.setHoodPosition(hoodPosition);
            }
            if (gamepad2.dpad_down) {
                selectedPreset = "6 FT";
                flywheelPower = TeleOpConstants.PRESET_6FT_POWER;
                hoodPosition = TeleOpConstants.PRESET_6FT_HOOD;
                launcher.setHoodPosition(hoodPosition);
            }
            if (gamepad2.dpad_left) {
                selectedPreset = "10 FT";
                flywheelPower = TeleOpConstants.PRESET_10FT_POWER;
                hoodPosition = TeleOpConstants.PRESET_10FT_HOOD;
                launcher.setHoodPosition(hoodPosition);
            }

            // === AUTO SHOOT (GP2 Y) - Toggle on/off ===
            if (gamepad2.y && !lastY) {
                if (!shooting) {
                    // Start shooting
                    shooting = true;
                    shootTimer.reset();
                    launcher.setPower(flywheelPower);
                    launcher.setSpinning(true);
                    flywheelOn = true;
                    launcher.setHoodPosition(hoodPosition);
                } else {
                    // Stop shooting
                    shooting = false;
                    launcher.setSpinning(false);
                    flywheelOn = false;
                    intakeTransfer.stopIntake();
                    intakeTransfer.transferDown();
                    transferUp = false;
                }
            }
            lastY = gamepad2.y;

            // Auto shoot sequence while shooting is on
            // Pulsed feeding with ramp cycling to let flywheel recover between shots
            if (shooting) {
                long ms = (long) shootTimer.milliseconds();
                long cycleTime = TeleOpConstants.FEED_DURATION_MS + TeleOpConstants.PAUSE_DURATION_MS;

                // Spin-up period - just flywheel, no feeding, ramp DOWN
                if (ms < TeleOpConstants.SPIN_UP_TIME_MS) {
                    intakeTransfer.stopIntake();
                    intakeTransfer.transferDown();
                    transferUp = false;
                }
                // Feeding period with pauses
                else {
                    long feedCycle = (ms - TeleOpConstants.SPIN_UP_TIME_MS) % cycleTime;
                    if (feedCycle < TeleOpConstants.FEED_DURATION_MS) {
                        // Feed phase - ramp up, intake on
                        intakeTransfer.transferUp();
                        intakeTransfer.startIntake();
                        transferUp = true;
                    } else {
                        // Pause phase - STOP INTAKE FIRST, then ramp down
                        intakeTransfer.stopIntake();
                        // Small delay before ramp down (first 200ms of pause = intake off, ramp still up)
                        if (feedCycle - TeleOpConstants.FEED_DURATION_MS > 200) {
                            intakeTransfer.transferDown();
                            transferUp = false;
                        }
                    }
                }
            }

            // === UPDATE ===
            launcher.update();

            // === TELEMETRY ===
            telemetry.addLine("=== BLUE TELEOP ===");
            telemetry.addData("Speed", speedMode == 1 ? "SLOW 40%" : speedMode == 2 ? "FAST 100%" : "NORMAL 70%");
            telemetry.addData("Preset", selectedPreset);
            telemetry.addData("Flywheel", flywheelOn ? "ON " + (int)(flywheelPower*100) + "%" : "OFF");
            telemetry.addData("Shooting", shooting ? "YES" : "NO");
            telemetry.addData("Turret", turretTracking ? (turret.isLocked() ? "LOCKED ON" : "TRACKING") : "MANUAL");
            telemetry.update();
        }

        // Shutdown
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
    }
}

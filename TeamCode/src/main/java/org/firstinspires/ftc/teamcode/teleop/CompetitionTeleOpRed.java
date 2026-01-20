package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.TeleOpConstants;

/**
 * RED ALLIANCE Competition TeleOp - Turret tracks AprilTag ID 24
 *
 * GP1: Drive (sticks)
 * GP2: Triggers = intake/eject, D-Pad = presets, Y (hold) = auto shoot
 * Turret auto-tracks continuously
 */
@TeleOp(name = "Competition TeleOp RED", group = "Competition")
public class CompetitionTeleOpRed extends LinearOpMode {

    private static final AllianceColor ALLIANCE = AllianceColor.RED;

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private TurretLockOptimized turret;
    private Limelight3A limelight;

    // Limelight constants for distance calculation
    private static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032; // 8 inches
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;
    private static final int IMAGE_WIDTH_PIXELS = 1280;
    private static final int IMAGE_HEIGHT_PIXELS = 720;

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
    private boolean lastB = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    // Speed mode: 0 = normal (70%), 1 = slow (40%), 2 = fast (100%)
    private int speedMode = 0;

    // Limelight distance tracking
    private double limelightDistance = -1.0;  // Current live distance
    private double lockedDistance = -1.0;     // Distance locked with B button


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

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(20);
            limelight.pipelineSwitch(3);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        telemetry.addLine("RED TeleOp Ready");
        telemetry.addLine(limelight != null ? "Limelight: OK" : "Limelight: NOT FOUND");
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
            double speedMult = (speedMode == 1) ? 0.35 : (speedMode == 2) ? 0.95 : 0.65;
            if (shooting) speedMult *= 0.85;  // Reduce drive power while flywheel spinning

            // Drive directly
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

            // === LIMELIGHT DISTANCE - Always updating ===
            limelightDistance = -1.0;
            if (limelight != null && limelight.isConnected()) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double taPercent = result.getTa();
                    if (taPercent > 0.0) {
                        double pixelArea = (taPercent / 100.0) * (IMAGE_WIDTH_PIXELS * IMAGE_HEIGHT_PIXELS);
                        double tagPixelHeight = Math.sqrt(pixelArea);
                        double focalPx = (IMAGE_HEIGHT_PIXELS / 2.0)
                                / Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV_DEGREES / 2.0));
                        double distanceMeters = (APRILTAG_REAL_HEIGHT_METERS * focalPx) / tagPixelHeight;
                        limelightDistance = distanceMeters * 3.28084; // Convert to feet
                    }
                }
            }

            // === B BUTTON - Lock distance and set power/hood ===
            if (gamepad2.b && !lastB) {
                if (limelightDistance > 0) {
                    lockedDistance = limelightDistance;
                    // Set power and hood based on distance ranges (continuous boundaries)
                    if (lockedDistance >= 2.37 && lockedDistance < 4.50) {
                        flywheelPower = 0.65;
                        hoodPosition = 0.65;
                        selectedPreset = "AUTO CLOSE";
                    } else if (lockedDistance >= 4.50 && lockedDistance < 6.20) {
                        flywheelPower = 0.75;
                        hoodPosition = 0.70;
                        selectedPreset = "AUTO MID";
                    } else if (lockedDistance >= 6.20 && lockedDistance <= 8.00) {
                        flywheelPower = 0.80;
                        hoodPosition = 0.75;
                        selectedPreset = "AUTO FAR";
                    } else {
                        selectedPreset = "OUT OF RANGE";
                    }
                    launcher.setHoodPosition(hoodPosition);
                }
            }
            lastB = gamepad2.b;

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
            // Simple: spin up, then ramp up and intake at full speed
            if (shooting) {
                long ms = (long) shootTimer.milliseconds();

                // Spin-up period - just flywheel, no feeding, ramp DOWN
                if (ms < TeleOpConstants.SPIN_UP_TIME_MS) {
                    intakeTransfer.stopIntake();
                    intakeTransfer.transferDown();
                    transferUp = false;
                }
                // After spin-up - ramp up, intake at full speed
                else {
                    intakeTransfer.transferUp();
                    intakeTransfer.startIntake(1.0);  // Full speed
                    transferUp = true;
                }
            }

            // === UPDATE ===
            launcher.update();

            // === TELEMETRY ===
            telemetry.addLine("=== RED TELEOP ===");
            telemetry.addData("Speed", speedMode == 1 ? "SLOW 35%" : speedMode == 2 ? "FAST 95%" : "NORMAL 65%");
            telemetry.addData("Preset", selectedPreset);
            telemetry.addData("Flywheel", flywheelOn ? "ON " + (int)(flywheelPower*100) + "%" : "OFF");
            telemetry.addData("Hood", "%.2f", hoodPosition);
            telemetry.addData("Shooting", shooting ? "YES" : "NO");
            telemetry.addData("Turret", turretTracking ? (turret.isLocked() ? "LOCKED ON" : "TRACKING") : "MANUAL");
            telemetry.addLine();
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Live Distance", limelightDistance > 0 ? String.format("%.2f ft", limelightDistance) : "NO TARGET");
            telemetry.addData("Locked Distance", lockedDistance > 0 ? String.format("%.2f ft", lockedDistance) : "NOT SET");
            telemetry.addLine();
            telemetry.addLine("=== DISTANCE RANGES ===");
            telemetry.addData("CLOSE", "2.37-4.49 ft | 65% | 0.65");
            telemetry.addData("MID", "4.50-6.19 ft | 75% | 0.70");
            telemetry.addData("FAR", "6.20-8.00 ft | 80% | 0.75");
            telemetry.addLine("B = Lock distance | Y = Shoot");
            telemetry.update();
        }

        // Shutdown
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
    }

}

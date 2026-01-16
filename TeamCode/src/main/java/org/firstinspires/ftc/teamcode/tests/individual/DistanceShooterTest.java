package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.components.LimelightLocalization;
import org.firstinspires.ftc.teamcode.core.components.TurretLockOptimized;

/**
 * Distance-Based Shooter Test
 *
 * Automatically adjusts hood angle and flywheel power based on
 * distance to AprilTag measured by Limelight.
 *
 * ============== CALIBRATION TABLE ==============
 * Adjust these values by testing at each distance!
 *
 * Distance (in) | Hood Pos | Flywheel Power
 * --------------|----------|---------------
 *    24 (2 ft)  |   0.85   |     0.50
 *    36 (3 ft)  |   0.80   |     0.55
 *    48 (4 ft)  |   0.75   |     0.60
 *    60 (5 ft)  |   0.70   |     0.65
 *    72 (6 ft)  |   0.65   |     0.70
 *    84 (7 ft)  |   0.60   |     0.75
 *    96 (8 ft)  |   0.55   |     0.80
 *   108 (9 ft)  |   0.50   |     0.85
 *   120 (10 ft) |   0.45   |     0.90
 *   132 (11 ft) |   0.40   |     0.95
 *   144 (12 ft) |   0.35   |     1.00
 *
 * ============== CONTROLS ==============
 * RIGHT TRIGGER: Spin up flywheel (hold)
 * LEFT TRIGGER:  Manual flywheel control
 * DPAD UP/DOWN:  Manual hood adjust
 * DPAD LEFT/RIGHT: Manual power adjust
 * A: Toggle auto mode (distance-based)
 * B: Save current settings to calibration
 * X: Cycle through test distances
 * Y: Reset adjustments
 * LEFT BUMPER: Toggle alliance (RED/BLUE)
 * RIGHT BUMPER: Lock/unlock turret manually
 *
 * ============== TEST PROCEDURE ==============
 * 1. Place robot at known distance from AprilTag
 * 2. Press X to select that distance in the table
 * 3. Use DPAD to adjust hood until shots are accurate
 * 4. Use triggers to find optimal flywheel power
 * 5. Press B to save the values
 * 6. Move to next distance and repeat
 */
@TeleOp(name = "Test: Distance Shooter", group = "Test")
public class DistanceShooterTest extends LinearOpMode {

    // ==================== CALIBRATION TABLE ====================
    // DISTANCES in inches (adjust hood & power for each)
    private static final double[] DISTANCES = {
        24,   // 2 feet
        36,   // 3 feet
        48,   // 4 feet
        60,   // 5 feet
        72,   // 6 feet
        84,   // 7 feet
        96,   // 8 feet
        108,  // 9 feet
        120,  // 10 feet
        132,  // 11 feet
        144   // 12 feet
    };

    // Hood positions for each distance (0.0 = down, 1.0 = up)
    // Lower hood = higher arc, for closer shots
    // Higher hood = flatter shot, for farther shots
    private double[] hoodPositions = {
        0.85,  // 2 feet - high arc
        0.80,  // 3 feet
        0.75,  // 4 feet
        0.70,  // 5 feet
        0.65,  // 6 feet
        0.60,  // 7 feet
        0.55,  // 8 feet
        0.50,  // 9 feet
        0.45,  // 10 feet
        0.40,  // 11 feet
        0.35   // 12 feet - flat shot
    };

    // Flywheel power for each distance (0.0 - 1.0)
    // Closer = less power, Farther = more power
    private double[] flywheelPowers = {
        0.50,  // 2 feet
        0.55,  // 3 feet
        0.60,  // 4 feet
        0.65,  // 5 feet
        0.70,  // 6 feet
        0.75,  // 7 feet
        0.80,  // 8 feet
        0.85,  // 9 feet
        0.90,  // 10 feet
        0.95,  // 11 feet
        1.00   // 12 feet
    };

    // ==================== HARDWARE ====================
    private LimelightLocalization localization;
    private TurretLockOptimized turret;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private Servo hoodServo;

    // ==================== STATE ====================
    private boolean autoMode = true;
    private int selectedIndex = 5;  // Start at 6 feet
    private double currentHoodPos = 0.5;
    private double currentFlywheelPower = 0.0;
    private double measuredDistance = -1;
    private AllianceColor alliance = AllianceColor.BLUE;

    // Manual adjustments
    private double manualHoodAdjust = 0.0;
    private double manualPowerAdjust = 0.0;

    // Button edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevLB = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {
        // Initialize localization
        localization = new LimelightLocalization();
        localization.initialize(hardwareMap, telemetry);

        // Initialize turret
        turret = new TurretLockOptimized();
        turret.initialize(hardwareMap, telemetry, alliance);

        // Initialize hardware
        flywheel1 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR);
        flywheel2 = hardwareMap.get(DcMotorEx.class, HardwareConfig.FLYWHEEL_MOTOR_2);
        hoodServo = hardwareMap.get(Servo.class, HardwareConfig.HOOD_SERVO);

        if (flywheel1 != null) {
            flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (flywheel2 != null) {
            flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel2.setDirection(DcMotor.Direction.REVERSE);
        }

        // Show init message
        telemetry.addLine("=== Distance Shooter Test ===");
        telemetry.addLine("");
        telemetry.addLine("Turret auto-tracks AprilTag");
        telemetry.addLine("Hood & power adjust by distance");
        telemetry.addLine("");
        telemetry.addLine("RT = Spin flywheel");
        telemetry.addLine("DPAD = Adjust hood/power");
        telemetry.addLine("A = Toggle auto mode");
        telemetry.addLine("LB = Toggle alliance");
        telemetry.addLine("RB = Lock/unlock turret");
        telemetry.addLine("");
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update turret tracking
            turret.update();

            // Update localization
            localization.update();
            measuredDistance = localization.getDistance();

            // Handle controls
            handleControls();

            // Calculate hood and power
            if (autoMode && measuredDistance > 0) {
                // Auto mode - interpolate based on measured distance
                currentHoodPos = interpolateHood(measuredDistance) + manualHoodAdjust;
                currentFlywheelPower = interpolatePower(measuredDistance) + manualPowerAdjust;
            } else {
                // Manual mode - use selected index
                currentHoodPos = hoodPositions[selectedIndex] + manualHoodAdjust;
                currentFlywheelPower = flywheelPowers[selectedIndex] + manualPowerAdjust;
            }

            // Clamp values
            currentHoodPos = Math.max(0.0, Math.min(1.0, currentHoodPos));
            currentFlywheelPower = Math.max(0.0, Math.min(1.0, currentFlywheelPower));

            // Apply hood position
            if (hoodServo != null) {
                hoodServo.setPosition(currentHoodPos);
            }

            // Apply flywheel power (only when trigger held)
            double flywheelOutput = 0;
            if (gamepad1.right_trigger > 0.1) {
                flywheelOutput = currentFlywheelPower;
            } else if (gamepad1.left_trigger > 0.1) {
                flywheelOutput = gamepad1.left_trigger;  // Manual power
            }

            if (flywheel1 != null) flywheel1.setPower(flywheelOutput);
            if (flywheel2 != null) flywheel2.setPower(flywheelOutput);

            // Display telemetry
            displayTelemetry(flywheelOutput);

            idle();
        }

        // Stop motors
        if (flywheel1 != null) flywheel1.setPower(0);
        if (flywheel2 != null) flywheel2.setPower(0);
    }

    private void handleControls() {
        // A: Toggle auto mode
        if (gamepad1.a && !prevA) {
            autoMode = !autoMode;
            manualHoodAdjust = 0;
            manualPowerAdjust = 0;
        }
        prevA = gamepad1.a;

        // X: Cycle through test distances
        if (gamepad1.x && !prevX) {
            selectedIndex = (selectedIndex + 1) % DISTANCES.length;
            manualHoodAdjust = 0;
            manualPowerAdjust = 0;
        }
        prevX = gamepad1.x;

        // Y: Reset hood to center
        if (gamepad1.y && !prevY) {
            manualHoodAdjust = 0;
            manualPowerAdjust = 0;
        }
        prevY = gamepad1.y;

        // B: Save current calibration
        if (gamepad1.b && !prevB) {
            saveCalibration();
        }
        prevB = gamepad1.b;

        // DPAD UP: Increase hood (flatter shot)
        if (gamepad1.dpad_up && !prevDpadUp) {
            manualHoodAdjust += 0.02;
        }
        prevDpadUp = gamepad1.dpad_up;

        // DPAD DOWN: Decrease hood (higher arc)
        if (gamepad1.dpad_down && !prevDpadDown) {
            manualHoodAdjust -= 0.02;
        }
        prevDpadDown = gamepad1.dpad_down;

        // DPAD LEFT/RIGHT: Adjust power
        if (gamepad1.dpad_left) {
            manualPowerAdjust -= 0.01;
        }
        if (gamepad1.dpad_right) {
            manualPowerAdjust += 0.01;
        }

        // Clamp manual adjustments
        manualHoodAdjust = Math.max(-0.3, Math.min(0.3, manualHoodAdjust));
        manualPowerAdjust = Math.max(-0.3, Math.min(0.3, manualPowerAdjust));

        // LEFT BUMPER: Toggle alliance
        if (gamepad1.left_bumper && !prevLB) {
            if (alliance == AllianceColor.BLUE) {
                alliance = AllianceColor.RED;
            } else {
                alliance = AllianceColor.BLUE;
            }
            turret.setAlliance(alliance);
        }
        prevLB = gamepad1.left_bumper;

        // RIGHT BUMPER: Lock/unlock turret
        if (gamepad1.right_bumper && !prevRB) {
            if (turret.isLocked()) {
                turret.unlock();
            } else {
                turret.lock();
            }
        }
        prevRB = gamepad1.right_bumper;
    }

    private void saveCalibration() {
        // Save current values to the selected index
        if (autoMode && measuredDistance > 0) {
            // Find closest distance index
            int idx = findClosestIndex(measuredDistance);
            hoodPositions[idx] = currentHoodPos;
            flywheelPowers[idx] = currentFlywheelPower;
        } else {
            hoodPositions[selectedIndex] = currentHoodPos;
            flywheelPowers[selectedIndex] = currentFlywheelPower;
        }
    }

    private int findClosestIndex(double distance) {
        int closest = 0;
        double minDiff = Math.abs(distance - DISTANCES[0]);
        for (int i = 1; i < DISTANCES.length; i++) {
            double diff = Math.abs(distance - DISTANCES[i]);
            if (diff < minDiff) {
                minDiff = diff;
                closest = i;
            }
        }
        return closest;
    }

    // ==================== INTERPOLATION ====================

    /**
     * Interpolate hood position based on distance
     */
    private double interpolateHood(double distance) {
        return interpolate(DISTANCES, hoodPositions, distance);
    }

    /**
     * Interpolate flywheel power based on distance
     */
    private double interpolatePower(double distance) {
        return interpolate(DISTANCES, flywheelPowers, distance);
    }

    /**
     * Linear interpolation between calibration points
     */
    private double interpolate(double[] xValues, double[] yValues, double x) {
        // Handle out of range
        if (x <= xValues[0]) return yValues[0];
        if (x >= xValues[xValues.length - 1]) return yValues[yValues.length - 1];

        // Find the two points to interpolate between
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {
                double t = (x - xValues[i]) / (xValues[i + 1] - xValues[i]);
                return yValues[i] + t * (yValues[i + 1] - yValues[i]);
            }
        }

        return yValues[0];
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry(double flywheelOutput) {
        telemetry.addLine("======== DISTANCE SHOOTER ========");
        telemetry.addLine("");

        // Turret status (prominent)
        String turretStatus = turret.getStateName();
        boolean readyToShoot = turret.isLocked() && measuredDistance > 0;
        telemetry.addData("TURRET", turretStatus + (turret.isLocked() ? " - LOCKED" : ""));
        telemetry.addData("READY TO SHOOT", readyToShoot ? "YES!" : "NO - " +
            (!turret.isLocked() ? "align turret" : "find tag"));
        telemetry.addLine("");

        // Mode & Alliance
        telemetry.addData("MODE", autoMode ? "AUTO (distance-based)" : "MANUAL (selected)");
        telemetry.addData("ALLIANCE", alliance.name() + " (Tag " + turret.getTargetTagId() + ")");
        telemetry.addLine("");

        // Distance
        telemetry.addLine("--- DISTANCE ---");
        if (measuredDistance > 0) {
            telemetry.addData("Measured", "%.1f in (%.2f ft)", measuredDistance, measuredDistance / 12.0);
        } else {
            telemetry.addData("Measured", "NO TAG VISIBLE");
        }
        telemetry.addData("Selected", "%.0f in (%.1f ft)", DISTANCES[selectedIndex], DISTANCES[selectedIndex] / 12.0);
        telemetry.addLine("");

        // Current settings
        telemetry.addLine("--- CURRENT SETTINGS ---");
        telemetry.addData("Hood Position", "%.3f", currentHoodPos);
        telemetry.addData("Target Power", "%.2f", currentFlywheelPower);
        telemetry.addData("Flywheel Output", "%.2f", flywheelOutput);
        telemetry.addLine("");

        // Manual adjustments
        if (manualHoodAdjust != 0 || manualPowerAdjust != 0) {
            telemetry.addLine("--- ADJUSTMENTS ---");
            telemetry.addData("Hood Adjust", "%+.3f", manualHoodAdjust);
            telemetry.addData("Power Adjust", "%+.2f", manualPowerAdjust);
            telemetry.addLine("");
        }

        // Calibration table
        telemetry.addLine("--- CALIBRATION TABLE ---");
        for (int i = 0; i < DISTANCES.length; i++) {
            String marker = (i == selectedIndex) ? ">>>" : "   ";
            String active = "";
            if (autoMode && measuredDistance > 0) {
                int closest = findClosestIndex(measuredDistance);
                if (i == closest) active = " <-- ACTIVE";
            }
            telemetry.addLine(String.format("%s %2.0fft: Hood=%.2f Pwr=%.2f%s",
                    marker, DISTANCES[i] / 12.0, hoodPositions[i], flywheelPowers[i], active));
        }
        telemetry.addLine("");

        // Turret info
        telemetry.addLine("--- TURRET ---");
        telemetry.addData("State", turret.getStateName());
        telemetry.addData("Servo Pos", "%.3f", turret.getServoPosition());
        telemetry.addData("tx", "%.1f°", turret.getTx());
        telemetry.addData("Tag Visible", turret.isTagVisible() ? "YES" : "NO");
        telemetry.addLine("");

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("RT=Shoot | DPAD=Adjust hood/pwr");
        telemetry.addLine("A=Mode | LB=Alliance | RB=Lock");
        telemetry.addLine("X=Cycle dist | B=Save | Y=Reset");

        telemetry.update();
    }

    // ==================== STATIC METHODS FOR OTHER CLASSES ====================

    /**
     * Get recommended hood position for a given distance
     * Can be called from other classes
     */
    public static double getHoodForDistance(double distanceInches) {
        // Default calibration values
        double[] distances = {24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144};
        double[] hoods = {0.85, 0.80, 0.75, 0.70, 0.65, 0.60, 0.55, 0.50, 0.45, 0.40, 0.35};

        if (distanceInches <= distances[0]) return hoods[0];
        if (distanceInches >= distances[distances.length - 1]) return hoods[hoods.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distanceInches >= distances[i] && distanceInches <= distances[i + 1]) {
                double t = (distanceInches - distances[i]) / (distances[i + 1] - distances[i]);
                return hoods[i] + t * (hoods[i + 1] - hoods[i]);
            }
        }
        return 0.5;
    }

    /**
     * Get recommended flywheel power for a given distance
     * Can be called from other classes
     */
    public static double getPowerForDistance(double distanceInches) {
        // Default calibration values
        double[] distances = {24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144};
        double[] powers = {0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00};

        if (distanceInches <= distances[0]) return powers[0];
        if (distanceInches >= distances[distances.length - 1]) return powers[powers.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distanceInches >= distances[i] && distanceInches <= distances[i + 1]) {
                double t = (distanceInches - distances[i]) / (distances[i + 1] - distances[i]);
                return powers[i] + t * (powers[i + 1] - powers[i]);
            }
        }
        return 0.7;
    }
}

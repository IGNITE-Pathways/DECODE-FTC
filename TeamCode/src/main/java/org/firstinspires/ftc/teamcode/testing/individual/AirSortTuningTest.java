package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Launcher;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.testing.DistanceCalculator;

import java.io.File;
import java.io.FileWriter;
import java.util.List;
import java.util.Locale;

/**
 * AirSort test OpMode (simple).
 *
 * Always:
 * - Continuously displays Limelight distance (same method as competition teleop "RANGES")
 * - Computes the NORMAL shooter preset from your existing range table (RobotConstants)
 *
 * When you press a button to run the test, it assumes 3 balls are preloaded and shoots:
 * 1) SLOWEST  = normal hood - slowestHoodDelta, normal rpm + slowestRpmDelta
 * 2) NORMAL
 * 3) NORMAL
 *
 * It also logs distance + settings to a CSV each time the 3-shot test is started.
 */
@TeleOp(name = "Test: AirSort Tuner", group = "Test")
public class AirSortTuningTest extends LinearOpMode {

    // =========================
    // Editable AirSort deltas (change these in code)
    // =========================

    /** SLOWEST shot: hood = normal - delta */
    public static double slowestHoodDelta = 0.445;
    /** SLOWEST shot: rpm = normal + delta */
    public static double slowestRpmDelta = 550.0;

    /** Tag selection (default BLUE). You can also toggle in-opmode. */
    public static AllianceColor allianceDefault = AllianceColor.BLUE;

    // Sequence timing / tolerance
    public static double rpmTolerance = 150.0;
    /**
     * How long to run the intake motor to feed exactly ONE ball.
     * If you see 2+ balls firing on shot 1, LOWER these.
     */
    public static long feedPulseShot1Ms = 320;
    public static long feedPulseShot2Ms = 320;
    public static long feedPulseShot3Ms = 360;
    /** Intake power during feed pulses (0-1). Lower helps prevent double-feeds. */
    public static double feedPulsePower = 1.00;

    /**
     * Optional anti-double-feed: briefly reverse after each feed pulse to pull the next ball back.
     * If your 3rd ball "disappears", increase reverseMs or decrease feedPulseMs.
     */
    public static long antiDoubleFeedReverseMs = 0;
    public static double antiDoubleFeedReversePower = 0.35;

    /** Minimum time to wait between shots (in addition to RPM tolerance check). */
    public static long recoverMinMs = 150;
    /**
     * Initial delay before feeding shot #1 after switching to SLOWEST.
     * This gives the hood time to move before the first ball is fed.
     */
    public static long preShot1DelayMs = 800;
    /** Extra wait after shot 1 before shot 2 can start (lets hood/RPM settle). */
    public static long postShot1DelayMs = 220;
    /** Extra wait after shot 2 before shot 3 can start (lets hood/RPM settle). */
    public static long postShot2DelayMs = 190;
    /** Extra "motor-off" dwell after each feed pulse (lets ball clear). */
    public static long postFeedDwellMs = 80;

    private static class NormalSettings {
        double rpm;
        double hood;
        String presetName;
    }

    // =========================
    // Hardware / Components
    // =========================

    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private Limelight3A limelight;

    // AprilTag IDs (same as CompetitionTeleOpBase)
    private static final int BLUE_APRILTAG = 20;
    private static final int RED_APRILTAG = 24;

    private AllianceColor alliance = allianceDefault;
    private int targetAprilTagId = BLUE_APRILTAG;

    // =========================
    // Distance state
    // =========================

    private double currentDistanceFeet = -1.0;
    private double lastGoodDistanceFeet = -1.0;
    private double currentBucketFeet = -1.0;

    // Flywheel
    private boolean flywheelOn = false;

    // =========================
    // 3-shot sequence state
    // =========================

    private enum SeqState {
        IDLE,
        WAIT_SPINUP,
        WAIT_PRE_SHOT_1,
        SHOT_1,
        RECOVER_1,
        SHOT_2,
        RECOVER_2,
        SHOT_3,
        DONE
    }

    private SeqState seqState = SeqState.IDLE;
    private final ElapsedTime seqTimer = new ElapsedTime();
    private String lastLogLine = "(none)";
    private int shotsCompleted = 0;

    // =========================
    // Button edge detection
    // =========================

    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastStart = false;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("=== AIRSORT TUNER READY ===");
        telemetry.addLine("Shows Limelight distance + NORMAL range preset.");
        telemetry.addLine("Shoots 3 balls: SLOWEST -> NORMAL -> NORMAL");
        telemetry.addLine("Buttons: Y=Flywheel  A=Run 3-shot  B=Cancel  START=Alliance");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Distance first (so telemetry + presets are correct)
            updateDistanceFeet();

            // Update launcher control loop
            launcher.update();

            handleControls();
            updateSequence();
            updateTelemetry();

            idle();
        }

        shutdown();
    }

    // =========================
    // Init / Shutdown
    // =========================

    private void initializeHardware() {
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);
        launcher.setVelocityControlEnabled(RobotConstants.USE_VELOCITY_CONTROL);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        intakeTransfer.transferDown();

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(20);
            limelight.pipelineSwitch(3); // AprilTag pipeline (matches CompetitionTeleOpBase)
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        setAlliance(alliance);
    }

    private void shutdown() {
        try {
            cancelSequence();
            launcher.setSpinning(false);
            launcher.update();
            intakeTransfer.stopIntake();
            intakeTransfer.transferDown();
        } catch (Exception ignored) {
            // No-op
        }
    }

    // =========================
    // Distance + presets
    // =========================

    private void updateDistanceFeet() {
        double dist = calculateCurrentDistanceFeet();
        currentDistanceFeet = dist;
        if (dist > 0) {
            lastGoodDistanceFeet = dist;
        }

        double basis = (dist > 0) ? dist : lastGoodDistanceFeet;
        if (basis > 0) {
            currentBucketFeet = bucketFeet(basis);
        } else {
            currentBucketFeet = -1.0;
        }
    }

    private double bucketFeet(double feet) {
        return Math.round(feet * 10.0) / 10.0; // 0.1ft buckets
    }

    private double calculateCurrentDistanceFeet() {
        if (limelight == null || !limelight.isConnected()) {
            return -1.0;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1.0;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return -1.0;
        }

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetAprilTagId) {
                return DistanceCalculator.calculateDistance(result); // same method used in teleop base
            }
        }

        return -1.0;
    }

    private NormalSettings getNormalSettings(double distanceFeet) {
        NormalSettings s = new NormalSettings();

        // Defaults
        s.rpm = RobotConstants.DEFAULT_TARGET_RPM;
        s.hood = RobotConstants.HOOD_DEFAULT_POSITION;
        s.presetName = "DEFAULT";

        if (distanceFeet <= 0 || Double.isNaN(distanceFeet)) {
            return s;
        }

        // Match CompetitionTeleOpBase.applyDistancePreset() ranges
        if (distanceFeet >= RobotConstants.RANGE_1_MIN && distanceFeet <= RobotConstants.RANGE_1_MAX) {
            s.rpm = RobotConstants.RANGE_1_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_1_HOOD_POSITION;
            s.presetName = "RANGE 1";
        } else if (distanceFeet >= RobotConstants.RANGE_2_MIN && distanceFeet <= RobotConstants.RANGE_2_MAX) {
            s.rpm = RobotConstants.RANGE_2_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_2_HOOD_POSITION;
            s.presetName = "RANGE 2";
        } else if (distanceFeet >= RobotConstants.RANGE_3_MIN && distanceFeet <= RobotConstants.RANGE_3_MAX) {
            s.rpm = RobotConstants.RANGE_3_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_3_HOOD_POSITION;
            s.presetName = "RANGE 3";
        } else if (distanceFeet >= RobotConstants.RANGE_4_MIN && distanceFeet <= RobotConstants.RANGE_4_MAX) {
            s.rpm = RobotConstants.RANGE_4_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_4_HOOD_POSITION;
            s.presetName = "RANGE 4";
        } else if (distanceFeet >= RobotConstants.RANGE_5_MIN && distanceFeet <= RobotConstants.RANGE_5_MAX) {
            s.rpm = RobotConstants.RANGE_5_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_5_HOOD_POSITION;
            s.presetName = "RANGE 5";
        } else if (distanceFeet >= RobotConstants.RANGE_6_MIN && distanceFeet <= RobotConstants.RANGE_6_MAX) {
            s.rpm = RobotConstants.RANGE_6_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_6_HOOD_POSITION;
            s.presetName = "RANGE 6";
        } else if (distanceFeet >= RobotConstants.RANGE_7_MIN && distanceFeet <= RobotConstants.RANGE_7_MAX) {
            s.rpm = RobotConstants.RANGE_7_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_7_HOOD_POSITION;
            s.presetName = "RANGE 7";
        } else if (distanceFeet >= RobotConstants.RANGE_FAR_MIN) {
            s.rpm = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            s.hood = RobotConstants.RANGE_FAR_HOOD_POSITION;
            s.presetName = "FAR";
        } else {
            s.presetName = "OUT OF RANGE -> DEFAULT";
        }

        return s;
    }

    // =========================
    // Controls
    // =========================

    private void handleControls() {
        // START toggles alliance/tag
        if (gamepad1.start && !lastStart) {
            setAlliance(alliance == AllianceColor.BLUE ? AllianceColor.RED : AllianceColor.BLUE);
        }
        lastStart = gamepad1.start;

        // Y toggles flywheel
        if (gamepad1.y && !lastY) {
            flywheelOn = !flywheelOn;
            launcher.setSpinning(flywheelOn);
            if (flywheelOn) {
                intakeTransfer.transferUp();
            } else {
                intakeTransfer.transferDown();
                cancelSequence();
            }
        }
        lastY = gamepad1.y;

        // A starts the 3-shot sequence
        if (gamepad1.a && !lastA) {
            if (seqState == SeqState.IDLE || seqState == SeqState.DONE) {
                startSequence();
            }
        }
        lastA = gamepad1.a;

        // B cancels
        if (gamepad1.b && !lastB) {
            cancelSequence();
        }
        lastB = gamepad1.b;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void setAlliance(AllianceColor newAlliance) {
        alliance = newAlliance;
        targetAprilTagId = (alliance == AllianceColor.BLUE) ? BLUE_APRILTAG : RED_APRILTAG;
    }

    // =========================
    // 3-shot sequence
    // =========================

    private void startSequence() {
        if (!flywheelOn) {
            // Auto-enable flywheel when starting sequence
            flywheelOn = true;
            launcher.setSpinning(true);
            intakeTransfer.transferUp();
        }
        seqState = SeqState.WAIT_SPINUP;
        seqTimer.reset();
        shotsCompleted = 0;

        appendLogLine();
    }

    private void cancelSequence() {
        seqState = SeqState.IDLE;
        intakeTransfer.stopIntake();
        seqTimer.reset();
        shotsCompleted = 0;
    }

    private void updateSequence() {
        if (seqState == SeqState.IDLE) {
            // Keep launcher set to NORMAL preset while idle
            applyNormal();
            return;
        }

        if (!flywheelOn) {
            cancelSequence();
            return;
        }

        // Always keep ramp up during sequence
        intakeTransfer.transferUp();

        switch (seqState) {
            case WAIT_SPINUP:
                // Spin up to shot #1 settings first
                applySlowest();
                if (rpmReady()) {
                    // Now wait a dedicated pre-shot delay so hood/RPM changes actually apply
                    seqState = SeqState.WAIT_PRE_SHOT_1;
                    seqTimer.reset();
                }
                break;

            case WAIT_PRE_SHOT_1:
                intakeTransfer.stopIntake();
                applySlowest();
                if (seqTimer.milliseconds() >= preShot1DelayMs && rpmReady()) {
                    seqState = SeqState.SHOT_1;
                    seqTimer.reset();
                }
                break;

            case SHOT_1:
                runShotPhase(ShotType.SLOWEST, feedPulseShot1Ms, SeqState.RECOVER_1);
                break;

            case RECOVER_1:
                // Ensure we are NOT feeding balls while changing config
                intakeTransfer.stopIntake();
                applyNormal(); // shot #2 settings
                long wait1 = Math.max(recoverMinMs, postShot1DelayMs);
                if (seqTimer.milliseconds() >= wait1 && rpmReady()) {
                    seqState = SeqState.SHOT_2;
                    seqTimer.reset();
                }
                break;

            case SHOT_2:
                runShotPhase(ShotType.NORMAL, feedPulseShot2Ms, SeqState.RECOVER_2);
                break;

            case RECOVER_2:
                // Ensure we are NOT feeding balls while changing config
                intakeTransfer.stopIntake();
                applyNormal(); // shot #3 settings
                long wait2 = Math.max(recoverMinMs, postShot2DelayMs);
                if (seqTimer.milliseconds() >= wait2 && rpmReady()) {
                    seqState = SeqState.SHOT_3;
                    seqTimer.reset();
                }
                break;

            case SHOT_3:
                runShotPhase(ShotType.NORMAL, feedPulseShot3Ms, SeqState.DONE);
                break;

            case DONE:
                intakeTransfer.stopIntake();
                // Stay DONE until cancelled or restarted
                break;
        }
    }

    private enum ShotType { NORMAL, SLOWEST }

    private void runShotPhase(ShotType type, long durationMs, SeqState nextState) {
        applyShot(type);
        double t = seqTimer.milliseconds();
        if (t < durationMs) {
            // FEED one ball
            intakeTransfer.startIntake(feedPulsePower);
        } else if (t < durationMs + antiDoubleFeedReverseMs) {
            // Optional reverse to prevent the next ball creeping in
            if (antiDoubleFeedReverseMs > 0) {
                intakeTransfer.startEject(antiDoubleFeedReversePower);
            } else {
                intakeTransfer.stopIntake();
            }
        } else if (t < durationMs + antiDoubleFeedReverseMs + postFeedDwellMs) {
            // Motor OFF dwell to let the ball clear
            intakeTransfer.stopIntake();
        } else {
            intakeTransfer.stopIntake();
            seqState = nextState;
            seqTimer.reset();
            shotsCompleted = Math.min(3, shotsCompleted + 1);
        }
    }

    private boolean rpmReady() {
        double target = launcher.getTargetRPM();
        double actual = launcher.getCurrentRPM();
        // IMPORTANT: allow overspeed to count as "ready"
        // (otherwise, when targets step DOWN for later shots, we can stall waiting for RPM to fall)
        return actual >= (target - rpmTolerance);
    }

    private void applyShot(ShotType type) {
        double distanceBasis = (currentDistanceFeet > 0) ? currentDistanceFeet : lastGoodDistanceFeet;
        NormalSettings normal = getNormalSettings(distanceBasis);

        double rpm = normal.rpm;
        double hood = normal.hood;

        if (type == ShotType.SLOWEST) {
            rpm = normal.rpm + slowestRpmDelta;
            hood = normal.hood - slowestHoodDelta;
        }

        // Clamp hood to servo range
        hood = clamp(hood, 0.0, 1.0);

        launcher.setTargetRPM(rpm);
        launcher.setHoodPosition(hood);
    }

    private void applyNormal() {
        applyShot(ShotType.NORMAL);
    }

    private void applySlowest() {
        applyShot(ShotType.SLOWEST);
    }

    // =========================
    // Telemetry
    // =========================

    private void updateTelemetry() {
        telemetry.addLine("=== AIRSORT (SIMPLE) ===");
        telemetry.addLine("Y=Flywheel  A=Run 3-shot  B=Cancel  START=Alliance");

        telemetry.addLine();
        telemetry.addLine("--- DISTANCE ---");
        if (currentDistanceFeet > 0) {
            telemetry.addData("Distance", "%.2f ft", currentDistanceFeet);
        } else {
            telemetry.addData("Distance", "NO TAG (last=%.2f ft)", lastGoodDistanceFeet);
        }
        telemetry.addData("Bucket", (currentBucketFeet > 0) ? String.format(Locale.US, "%.1f ft", currentBucketFeet) : "N/A");
        telemetry.addData("Alliance/Tag", "%s (ID %d)", alliance.name(), targetAprilTagId);

        double basis = (currentDistanceFeet > 0) ? currentDistanceFeet : lastGoodDistanceFeet;
        NormalSettings normal = getNormalSettings(basis);

        double slowestRpm = normal.rpm + slowestRpmDelta;
        double slowestHood = clamp(normal.hood - slowestHoodDelta, 0.0, 1.0);

        telemetry.addLine();
        telemetry.addLine("--- SETTINGS (from NORMAL ranges) ---");
        telemetry.addData("Normal", "%s  rpm=%.0f hood=%.3f", normal.presetName, normal.rpm, normal.hood);
        telemetry.addData("Slowest", "rpm=%.0f hood=%.3f  (hood-%.2f rpm+%.0f)", slowestRpm, slowestHood, slowestHoodDelta, slowestRpmDelta);

        telemetry.addLine();
        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        if (flywheelOn) {
            telemetry.addData("Target/Actual", "%.0f / %.0f RPM", launcher.getTargetRPM(), launcher.getCurrentRPM());
            telemetry.addData("Hood", "%.3f", launcher.getHoodPosition());
        }
        telemetry.addData("Ramp", intakeTransfer.isTransferUp() ? "UP" : "DOWN");

        telemetry.addLine();
        telemetry.addLine("--- 3-SHOT SEQUENCE ---");
        telemetry.addData("State", seqState.name());
        telemetry.addData("Shots completed", "%d/3", shotsCompleted);
        telemetry.addData("RPM tol", "%.0f", rpmTolerance);
        telemetry.addData("Delays", "pre1=%dms  post1=%dms  post2=%dms", preShot1DelayMs, postShot1DelayMs, postShot2DelayMs);
        telemetry.addData("Feed pulses", "%d / %d / %d ms", feedPulseShot1Ms, feedPulseShot2Ms, feedPulseShot3Ms);
        telemetry.addData("Feed power", "%.2f", feedPulsePower);
        telemetry.addData("Anti double-feed", "rev=%dms @ %.2f", antiDoubleFeedReverseMs, antiDoubleFeedReversePower);
        telemetry.addData("Post-feed dwell", "%d ms", postFeedDwellMs);
        if (flywheelOn) {
            telemetry.addData("RPM ready?", "%s (target %.0f actual %.0f)",
                    rpmReady() ? "YES" : "no",
                    launcher.getTargetRPM(),
                    launcher.getCurrentRPM());
        }
        telemetry.addData("Last log", lastLogLine);

        telemetry.update();
    }

    // =========================
    // Logging (CSV)
    // =========================

    private File getLogFile() {
        return AppUtil.getInstance().getSettingsFile("air_sort_fixed_offsets_log.csv");
    }

    private void appendLogLine() {
        File f = getLogFile();
        if (f == null) return;

        double basis = (currentDistanceFeet > 0) ? currentDistanceFeet : lastGoodDistanceFeet;
        double bucket = (basis > 0) ? bucketFeet(basis) : -1.0;
        NormalSettings normal = getNormalSettings(basis);

        double slowestRpm = normal.rpm + slowestRpmDelta;
        double slowestHood = clamp(normal.hood - slowestHoodDelta, 0.0, 1.0);

        boolean newFile = !f.exists();

        try (FileWriter w = new FileWriter(f, true)) {
            if (newFile) {
                w.write("timestampMs,alliance,tagId,distanceFeet,bucketFeet,normalPreset,normalRpm,normalHood,slowestRpm,slowestHood,slowestHoodDelta,slowestRpmDelta\n");
            }

            long ts = System.currentTimeMillis();
            String row = String.format(Locale.US,
                    "%d,%s,%d,%.3f,%.1f,%s,%.0f,%.4f,%.0f,%.4f,%.3f,%.0f\n",
                    ts,
                    alliance.name(),
                    targetAprilTagId,
                    basis,
                    bucket,
                    normal.presetName.replace(',', '_'),
                    normal.rpm,
                    normal.hood,
                    slowestRpm,
                    slowestHood,
                    slowestHoodDelta,
                    slowestRpmDelta
            );
            w.write(row);
            w.flush();

            lastLogLine = String.format(Locale.US, "Logged %.2fft (bucket %.1f)", basis, bucket);
        } catch (Exception ignored) {
            // No-op
        }
    }
}


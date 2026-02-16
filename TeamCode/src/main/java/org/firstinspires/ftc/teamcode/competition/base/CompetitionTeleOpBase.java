package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.components.DriveTrain;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.ActualTurretLockOn;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.util.DistanceCalculator;

/**
 * *** COMPETITION TELEOP ***
 *
 * ALL SETTINGS (PIDF gains, RPM ranges, hood positions) are in RobotConstants.java!
 * After tuning, update values there - they automatically apply here.
 *
 * === DEFAULT PRESET ===
 * Default: CLOSE RANGE (2400 RPM, hood 0.55) - Range 1 for close shots (2.45-3.45 ft)
 *
 * === FLYWHEEL & RAMP CONTROL ===
 * - GP1-Y: Toggle flywheel ON/OFF
 * - Ramp automatically raises when:
 *   1. Flywheel reaches target RPM (within 150 RPM) AND
 *   2. Robot is inside LAUNCH ZONE (triangle: (0,0), (0,144), (144,144) with 30" tolerance)
 * - Ramp automatically lowers when flywheel turns OFF OR robot leaves launch zone
 * - You can intake/cycle balls while flywheel is spinning up!
 *
 * === AUTOMATIC DISTANCE CONTROL (DEFAULT) ===
 * - Limelight automatically detects AprilTag distance (always on by default)
 * - Shooting preset (RPM & hood) automatically adjusts based on distance
 * - No button press needed - just aim and shoot!
 *
 * === GAMEPAD 1 (Driver/Shooter/Presets) ===
 * Sticks: Drive (100% speed always)
 * Y: Toggle flywheel ON/OFF (ramp auto-raises when RPM ready AND in launch zone)
 * RB: Start auto-shoot 3-ball sequence (when flywheel ON) | Press during auto = Cancel
 * LB: Eject-and-shoot sequence (gentle eject 400ms, then shoot 800ms)
 * RT: Intake (works even when flywheel ON!) | LT: Eject
 * DPAD UP: Re-enable limelight auto-detection (DEFAULT MODE)
 * DPAD DOWN: Disable distance detection (use close range default)
 * DPAD RIGHT: Far zone preset (10+ ft, 3450 RPM)
 *
 * === GAMEPAD 2 (Turret & Manual Controls) ===
 * A: Turret auto-track (aims at AprilTag)
 * X: Center turret
 * Left Stick X: Manual turret control
 * DPAD LEFT: Fine turret adjust left
 * DPAD UP: Increase RPM (+100)
 * DPAD DOWN: Decrease RPM (-100)
 * LB: Lower hood angle (-0.03)
 * RB: Raise hood angle (+0.03)
 * START: EMERGENCY STOP
 *
 * === Shooting Workflow ===
 * 1. GP2-A: Enable turret tracking (aims at AprilTag)
 * 2. Limelight automatically adjusts shooting preset based on distance
 * 3. GP1-Y: Turn ON flywheel (starts spinning up)
 * 4. Drive into LAUNCH ZONE (triangle zone in positive quadrant)
 * 5. Wait for ramp to auto-raise (when flywheel within 150 RPM AND in launch zone)
 * 6. GP1-RT: Intake balls or GP1-RB: Auto-shoot 3 balls
 * 7. Repeat as needed!
 *
 * === Quick Eject-Shoot (GP1-LB) ===
 * Use when ball is stuck or needs repositioning before shooting
 * - Automatically switches to SHOOTING mode if needed
 * - Gentle eject (35% power, 400ms) to reposition ball properly
 * - Then shoots for 800ms
 */
public abstract class CompetitionTeleOpBase extends LinearOpMode {

    // === STATE MACHINE ===
    private enum RobotState {
        INTAKE,   // Full speed, intake active
        SHOOTING  // Reduced speed, flywheel active
    }

    // === CONFIG ===
    private static final boolean SHOW_DEBUG_INFO = false;  // Set true to see limelight debug info
    private static final boolean SHOW_ZONE_DEBUG = true;   // Set true to see detailed zone check info

    // AprilTag IDs for distance tracking
    private static final int BLUE_APRILTAG = 20;
    private static final int RED_APRILTAG = 24;

    protected AllianceColor alliance;
    private int targetAprilTagId;  // Set based on alliance color
    private RobotState currentState = RobotState.INTAKE;  // Start in intake mode

    // Components
    private DriveTrain driveTrain;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;
    private ActualTurretLockOn turret;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // State
    private boolean flywheelOn = false;
    private boolean rampUp = false;  // Current ramp position (up/down)
    private boolean rampAutoControlEnabled = false;  // Auto-control feature toggle
    private double flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;  // Loaded from ShooterConstants
    private double hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
    private String selectedPreset = "DEFAULT";
    private boolean turretTracking = false; // Field-goal auto align default OFF (enable with right bumper)
    private boolean distanceDetectionEnabled = true;  // Default: ON (always detecting)
    private boolean emergencyStopped = false;

    // ==================== FIELD GOAL AUTO ALIGN CONFIG ====================
    // Pedro field coordinates (inches/degrees). Adjust to match your real start pose.
    private static final double FIELD_START_X_IN = 56.0;
    private static final double FIELD_START_Y_IN = 38.0;
    private static final double FIELD_START_HEADING_PEDRO_DEG = 90.0;

    // Gamepad 1 button states
    private boolean lastGP1_Y = false;
    private boolean lastGP1_LeftBumper = false;
    private boolean lastGP1_RightBumper = false;
    private boolean lastGP1_DpadUp = false;
    private boolean lastGP1_DpadDown = false;
    private boolean lastGP1_DpadRight = false;

    // Eject-and-shoot sequence
    private boolean ejectShootActive = false;
    private long ejectShootStartTime = 0;
    private static final long EJECT_DURATION_MS = 400;  // Eject for 400ms to position ball properly

    // Auto-shoot sequence (triggered by Y button in SHOOTING mode) - 3 BALLS
    private boolean autoShootActive = false;
    private boolean waitingForFlywheelSpinup = false;
    private int autoShootPhase = 0;  // 0=initial wait, 1=shot1, 2=wait, 3=shot2, 4=wait, 5=shot3
    private ElapsedTime autoShootTimer = new ElapsedTime();
    private static final double RPM_TOLERANCE = 150;  // Within 150 RPM to raise ramp and start shooting
    private static final double RPM_HYSTERESIS = 300; // Once ramp is up, allow 300 RPM error before lowering (prevents oscillation)
    private static final double SHOT_DURATION_MS = 500;   // 0.5 seconds for shots 1 & 2
    private static final double FINAL_SHOT_DURATION_MS = 1000;  // 1.0 second for final shot (ball 3)

    // Limelight distance tracking
    private double currentDistance = -1.0;  // Currently detected distance
    private double lockedDistance = -1.0;   // Locked distance for shooting

    // Pinpoint position tracking
    private double robotX = 0.0;  // Robot X position in inches
    private double robotY = 0.0;  // Robot Y position in inches
    private double robotHeadingPinpointDeg = 0.0; // Pinpoint heading (degrees)
    private boolean inLaunchZone = false;  // Is robot currently in a launch zone?

    // Gamepad 2 button states (manual turret & overrides)
    private boolean lastGP2_A = false;
    private boolean lastGP2_B = false;
    private boolean lastGP2_X = false;
    private boolean lastGP2_DpadLeft = false;
    private boolean lastGP2_DpadRight = false;
    private boolean lastGP2_DpadUp = false;
    private boolean lastGP2_DpadDown = false;
    private boolean lastGP2_LeftBumper = false;
    private boolean lastGP2_RightBumper = false;
    private boolean lastGP2_Y = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            // Emergency stop check (GP2-START)
            handleEmergencyStop();

            if (!emergencyStopped) {
                // Main control loop
                updateRobotPosition();        // Update position from Pinpoint
                handleDriving();              // GP1: Sticks, speed control
                handleTurret();               // GP2-A: Auto-tracking toggle
                handleManualTurretControl();  // GP2: Manual turret control
                handleIntake();               // GP1: RT/LT intake/eject
                handleEjectShoot();           // GP1-LB: Eject-and-shoot sequence
                handleDistanceLock();         // GP1: DPAD DOWN/RIGHT/UP, auto-apply distance
                handleFlywheelToggle();       // GP1-Y: Toggle flywheel ON/OFF (auto raises ramp)
                handleAutomaticRampControl(); // Auto-raise ramp when RPM ready AND in launch zone
                handleAutoShootTrigger();     // GP1-RB: Trigger auto-shoot sequence
                handleAutoShootSequence();    // Auto-shoot sequence (3-ball)
                handleManualOverrides();      // GP2: Manual hood adjustments
                launcher.update();            // PIDF velocity control
            }

            updateTelemetry();
        }

        shutdown();
    }

    // ========================================
    // INITIALIZATION
    // ========================================

    private void initializeHardware() {
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        // Apply driver comfort settings from ShooterConstants
        driveTrain.setInputCurve(RobotConstants.DRIVE_INPUT_CURVE);
        driveTrain.setRotationSensitivity(RobotConstants.ROTATION_SENSITIVITY);
        driveTrain.setBatteryCompensationEnabled(RobotConstants.ENABLE_BATTERY_COMPENSATION);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        // PIDF velocity control enabled/disabled from ShooterConstants
        // Tune gains using FlywheelPIDFTuner, then update RobotConstants.java
        launcher.setVelocityControlEnabled(RobotConstants.USE_VELOCITY_CONTROL);
        // Explicitly apply current tuned PIDF gains from RobotConstants
        launcher.setPIDFGains(
                RobotConstants.FLYWHEEL_KP,
                RobotConstants.FLYWHEEL_KI,
                RobotConstants.FLYWHEEL_KD,
                RobotConstants.FLYWHEEL_KF
        );

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        turret = new ActualTurretLockOn();
        turret.initialize(hardwareMap, telemetry, alliance);
        // Field-goal auto align (odometry + heading) aiming at alliance goal coords.
        turret.setFieldStartPosePedro(FIELD_START_X_IN, FIELD_START_Y_IN, FIELD_START_HEADING_PEDRO_DEG);
        turret.enableFieldGoalAutoAlign(alliance);

        // Set target AprilTag ID based on alliance color
        targetAprilTagId = (alliance == AllianceColor.BLUE) ? BLUE_APRILTAG : RED_APRILTAG;

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(20);
            limelight.pipelineSwitch(3);  // Pipeline 3 for AprilTag detection
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        // Initialize Pinpoint odometry
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareConfig.IMU);
            configurePinpoint();
        } catch (Exception e) {
            pinpoint = null;
        }

        telemetry.addLine(alliance.name() + " TeleOp Ready");
        telemetry.addLine(limelight != null ? "Limelight: OK" : "Limelight: NOT FOUND");
        telemetry.addLine(pinpoint != null ? "Pinpoint: OK" : "Pinpoint: NOT FOUND");
        telemetry.addLine("Turret: Field-goal auto align ON (GP2-A toggles)");
        telemetry.update();
    }

    private void configurePinpoint() {
        if (pinpoint == null) return;

        // Set odometry pod offsets (from RobotConstants)
        pinpoint.setOffsets(RobotConstants.PINPOINT_STRAFE_POD_X,
                           RobotConstants.PINPOINT_FORWARD_POD_Y,
                           DistanceUnit.INCH);

        // Set encoder type (GoBilda 4-bar pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();
    }

    // GP2-START: Emergency stop (hold to stop all motors, release to resume)
    private void handleEmergencyStop() {
        if (gamepad2.start) {
            if (!emergencyStopped) {
                emergencyStopped = true;
                stopAllMotors();
            }
        } else {
            emergencyStopped = false;
        }
    }

    private void stopAllMotors() {
        launcher.setSpinning(false);
        launcher.setTargetRPM(0);
        intakeTransfer.stopIntake();
        intakeTransfer.transferDown();
        driveTrain.stopMotors();
        turret.stop();
        turretTracking = false;
        flywheelOn = false;
        rampUp = false;  // Reset ramp state
    }

    // ========================================
    // POSITION TRACKING
    // ========================================
    // Launch zone is a triangle with vertices:
    // - (0, 0) - Bottom left
    // - (0, 144) - Top left
    // - (144, 144) - Top right
    // Includes 30-inch tolerance for odometry errors and skipping

    private void updateRobotPosition() {
        if (pinpoint == null) {
            inLaunchZone = true;  // Allow shooting if Pinpoint unavailable
            return;
        }

        // Update Pinpoint
        pinpoint.update();

        // Get current position
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeadingPinpointDeg = pose.getHeading(AngleUnit.DEGREES);

        // Prime offsets for field-goal align ASAP (does NOT move turret)
        boolean pinpointReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        turret.primeFieldGoalOffsetsFromPinpointPose(robotX, robotY, robotHeadingPinpointDeg, pinpointReady);

        // Check if robot is in the launch zone (triangular zone with 10" tolerance)
        inLaunchZone = RobotConstants.isInLaunchZone(robotX, robotY);
    }

    // ========================================
    // GAMEPAD 1: DRIVING & INTAKE
    // ========================================

    private void handleDriving() {
        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;

        // Apply joystick deadzone
        fwd = applyDeadzone(fwd);
        str = applyDeadzone(str);
        rot = applyDeadzone(rot);

        // Apply input curve to translation vector magnitude (preserves direction)
        double magnitude = Math.sqrt(fwd * fwd + str * str);
        if (magnitude > 0.01) {
            double curvedMagnitude = Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
            double scale = curvedMagnitude / magnitude;
            fwd *= scale;
            str *= scale;
        }

        // Apply input curve to rotation separately
        rot = applyInputCurve(rot);

        // Apply rotation sensitivity
        rot *= RobotConstants.ROTATION_SENSITIVITY;

        // Full speed for both modes - move fast during game
        double speedMult = 1.0;  // 100% speed always

        driveTrain.driveRaw(fwd * speedMult, str * speedMult, rot * speedMult);
    }

    private double applyInputCurve(double input) {
        if (input == 0) return 0;
        double sign = Math.signum(input);
        double magnitude = Math.abs(input);
        return sign * Math.pow(magnitude, RobotConstants.DRIVE_INPUT_CURVE);
    }

    private double applyDeadzone(double input) {
        if (Math.abs(input) < RobotConstants.JOYSTICK_DEAD_ZONE) {
            return 0;
        }
        // Rescale so that full stick = 1.0 (not 0.95)
        // This ensures 100% stick = 100% motor power
        double sign = Math.signum(input);
        return sign * (Math.abs(input) - RobotConstants.JOYSTICK_DEAD_ZONE) / (1.0 - RobotConstants.JOYSTICK_DEAD_ZONE);
    }

    // GP2-RB: Toggle turret auto-align to field goal (odometry + heading)
    private void handleTurret() {
        if (gamepad2.right_bumper && !lastGP2_RightBumper) {
            turretTracking = !turretTracking;
        }
        lastGP2_RightBumper = gamepad2.right_bumper;

        if (turretTracking) {
            // Field-goal auto align uses Pinpoint pose (exact trig implementation from the test).
            // (Limelight PID auto-track remains available if you disable field-goal mode.)
            if (turret.isFieldGoalAutoAlignEnabled()) {
                boolean pinpointReady = (pinpoint != null) && (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);
                turret.updateFieldGoalFromPinpointPose(robotX, robotY, robotHeadingPinpointDeg, pinpointReady);
            } else {
                turret.update();  // Limelight AprilTag tracking
            }
        }
    }

    // GP2: Manual turret control (only when auto-tracking is OFF)
    private void handleManualTurretControl() {
        if (turretTracking) {
            return;  // Auto-tracking takes priority
        }

        double currentPos = turret.getServoPosition();

        // Gamepad 2 Left Stick X: Continuous manual control
        double stickInput = -gamepad2.left_stick_x;  // Inverted for intuitive control
        if (Math.abs(stickInput) > RobotConstants.TURRET_MANUAL_STICK_DEADZONE) {
            double adjustment = stickInput * RobotConstants.TURRET_MANUAL_STICK_SENSITIVITY;
            currentPos += adjustment;
        }

        // Gamepad 2 DPAD Left: Small increment left
        if (gamepad2.dpad_left && !lastGP2_DpadLeft) {
            currentPos -= RobotConstants.TURRET_MANUAL_INCREMENT;
        }
        lastGP2_DpadLeft = gamepad2.dpad_left;

        // Gamepad 2 X: Reset to center
        if (gamepad2.x && !lastGP2_X) {
            currentPos = RobotConstants.TURRET_CENTER_POSITION;
        }
        lastGP2_X = gamepad2.x;

        // Clamp position to valid range
        currentPos = Math.max(RobotConstants.TURRET_MIN_POSITION,
                Math.min(RobotConstants.TURRET_MAX_POSITION, currentPos));

        // Apply position
        turret.setPositionDirect(currentPos);
    }

    // GP1: RT = Intake, LT = Eject
    private void handleIntake() {
        // Don't allow manual intake during auto sequences
        if (ejectShootActive || autoShootActive) {
            return;
        }

        if (gamepad1.right_trigger > RobotConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > RobotConstants.TRIGGER_DEADZONE) {
            intakeTransfer.startEject(gamepad1.left_trigger);
        } else {
            intakeTransfer.stopIntake();
        }
    }

    // GP1-LB: Eject ball halfway then shoot
    private void handleEjectShoot() {
        // Start sequence on button press
        if (gamepad1.left_bumper && !lastGP1_LeftBumper) {
            ejectShootActive = true;
            ejectShootStartTime = System.currentTimeMillis();

            // Ensure flywheel is ready (ramp will auto-raise when RPM ready)
            if (!flywheelOn) {
                currentState = RobotState.SHOOTING;
                activateFlywheel();
                rampAutoControlEnabled = true;
            }
        }
        lastGP1_LeftBumper = gamepad1.left_bumper;

        // Execute sequence
        if (ejectShootActive) {
            long elapsed = System.currentTimeMillis() - ejectShootStartTime;

            if (elapsed < EJECT_DURATION_MS) {
                // Phase 1: Eject to position ball (reverse intake at reduced power)
                intakeTransfer.startEject(0.35);  // Low power eject to just reposition (was 1.0)
            } else if (elapsed < EJECT_DURATION_MS + 800) {
                // Phase 2: Shoot (forward intake to feed into flywheel)
                intakeTransfer.startIntake(1.0);  // Full power shoot
            } else {
                // Sequence complete
                intakeTransfer.stopIntake();
                ejectShootActive = false;
            }
        }
    }

    // ========================================
    // GAMEPAD 2: LAUNCHER CONTROLS
    // ========================================

    // GP1 DPAD: Distance presets (auto-applies based on limelight)
    private void handleDistanceLock() {
        // DPAD UP: Re-enable limelight distance detection (DEFAULT)
        if (gamepad1.dpad_up && !lastGP1_DpadUp) {
            distanceDetectionEnabled = true;
            selectedPreset = "LIMELIGHT AUTO";
        }
        lastGP1_DpadUp = gamepad1.dpad_up;

        // DPAD DOWN: Disable detection, use default preset
        if (gamepad1.dpad_down && !lastGP1_DpadDown) {
            distanceDetectionEnabled = false;
            currentDistance = -1.0;
            lockedDistance = -1.0;
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
            selectedPreset = "DEFAULT";
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
                launcher.setHoodPosition(hoodPosition);
            }
        }
        lastGP1_DpadDown = gamepad1.dpad_down;

        // DPAD RIGHT: Quick far zone preset (10+ ft, max distance)
        if (gamepad1.dpad_right && !lastGP1_DpadRight) {
            distanceDetectionEnabled = false;
            lockedDistance = RobotConstants.RANGE_FAR_MIN;
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
            selectedPreset = "FAR ZONE (Manual)";
            turret.setPositionDirect(0.5);  // Set turret to center position
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
                launcher.setHoodPosition(hoodPosition);
            }
        }
        lastGP1_DpadRight = gamepad1.dpad_right;

        // Continuously read distance from limelight and automatically apply preset
        if (distanceDetectionEnabled) {
            double distance = calculateCurrentDistance();
            if (distance > 0) {
                // Only update if distance changed significantly (avoid jitter)
                if (Math.abs(distance - currentDistance) > 0.1) {  // 0.1 ft threshold
                    currentDistance = distance;
                    lockedDistance = distance;
                    applyDistancePreset(distance);  // Auto-apply preset for current distance
                }
            }
        }
    }

    private double calculateCurrentDistance() {
        if (limelight == null) {
            return -1.0;
        }

        if (!limelight.isConnected()) {
            return -1.0;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1.0;
        }

        // Filter for the correct AprilTag based on alliance color
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return -1.0;
        }

        // Find our target tag (Blue = 20, Red = 24)
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == targetAprilTagId) {
                // Found the correct tag! Calculate distance from it
                // Note: DistanceCalculator uses the overall result's ta (area)
                // For more accuracy, we could calculate from the specific fiducial's data
                return DistanceCalculator.calculateDistance(result);
            }
        }

        // Correct AprilTag not visible
        return -1.0;
    }

    // Debug info for limelight
    private String getLimelightDebugInfo() {
        if (limelight == null) {
            return "Limelight: NULL";
        }
        if (!limelight.isConnected()) {
            return "Limelight: NOT CONNECTED";
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return "Limelight: NO RESULT";
        }
        if (!result.isValid()) {
            return "Limelight: INVALID RESULT";
        }

        double ta = result.getTa();
        return String.format("Limelight OK | TA: %.2f%%", ta);
    }

    private void applyDistancePreset(double distance) {
        // All range values loaded from RobotConstants - update there after testing!
        // Full coverage: 2.45-3.45ft, 3.46-4.65ft, 4.66-5.35ft, 5.36-6ft, 6-7ft, 7-8.5ft, 8.5-10ft, 10+ft

        if (distance >= RobotConstants.RANGE_1_MIN && distance <= RobotConstants.RANGE_1_MAX) {
            // Range 1: 2.45 - 3.45 ft
            flywheelRPM = RobotConstants.RANGE_1_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_1_HOOD_POSITION;
            selectedPreset = String.format("RANGE 1 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_2_MIN && distance <= RobotConstants.RANGE_2_MAX) {
            // Range 2: 3.46 - 4.65 ft
            flywheelRPM = RobotConstants.RANGE_2_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_2_HOOD_POSITION;
            selectedPreset = String.format("RANGE 2 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_3_MIN && distance <= RobotConstants.RANGE_3_MAX) {
            // Range 3: 4.66 - 5.35 ft
            flywheelRPM = RobotConstants.RANGE_3_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_3_HOOD_POSITION;
            selectedPreset = String.format("RANGE 3 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_4_MIN && distance <= RobotConstants.RANGE_4_MAX) {
            // Range 4: 5.36 - 6.00 ft
            flywheelRPM = RobotConstants.RANGE_4_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_4_HOOD_POSITION;
            selectedPreset = String.format("RANGE 4 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_5_MIN && distance <= RobotConstants.RANGE_5_MAX) {
            // Range 5: 6.01 - 7.00 ft
            flywheelRPM = RobotConstants.RANGE_5_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_5_HOOD_POSITION;
            selectedPreset = String.format("RANGE 5 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_6_MIN && distance <= RobotConstants.RANGE_6_MAX) {
            // Range 6: 7.01 - 8.50 ft
            flywheelRPM = RobotConstants.RANGE_6_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_6_HOOD_POSITION;
            selectedPreset = String.format("RANGE 6 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_7_MIN && distance <= RobotConstants.RANGE_7_MAX) {
            // Range 7: 8.51 - 10.00 ft
            flywheelRPM = RobotConstants.RANGE_7_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_7_HOOD_POSITION;
            selectedPreset = String.format("RANGE 7 (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else if (distance >= RobotConstants.RANGE_FAR_MIN) {
            // Far range: 10+ ft
            flywheelRPM = RobotConstants.RANGE_FAR_FLYWHEEL_RPM;
            hoodPosition = RobotConstants.RANGE_FAR_HOOD_POSITION;
            turret.setPositionDirect(0.5);  // Set turret to center position
            selectedPreset = String.format("FAR ZONE (%.2fft @ %.0fRPM)", distance, flywheelRPM);

        } else {
            // Below 2.45 ft or invalid distance - use defaults
            flywheelRPM = RobotConstants.DEFAULT_TARGET_RPM;
            hoodPosition = RobotConstants.HOOD_DEFAULT_POSITION;
            selectedPreset = String.format("OUT OF RANGE (%.2fft) - DEFAULT", distance);
        }

        // Apply immediately if flywheel is already on
        if (flywheelOn) {
            launcher.setTargetRPM(flywheelRPM);
            launcher.setHoodPosition(hoodPosition);
        }
    }

    // GP1-Y: Toggle flywheel ON/OFF (ramp raises automatically when RPM ready)
    // Turns on flywheel and enters SHOOTING mode, or turns off and returns to INTAKE mode
    private void handleFlywheelToggle() {
        if (gamepad1.y && !lastGP1_Y) {
            if (currentState == RobotState.INTAKE) {
                // Switch to SHOOTING mode
                currentState = RobotState.SHOOTING;
                activateFlywheel();
                // Enable automatic ramp control (will raise when RPM is within 150 RPM)
                rampAutoControlEnabled = true;
            } else if (currentState == RobotState.SHOOTING) {
                // Turn off flywheel and return to INTAKE mode
                if (autoShootActive) {
                    cancelAutoShoot();
                }
                currentState = RobotState.INTAKE;
                deactivateFlywheel();
            }
        }
        lastGP1_Y = gamepad1.y;
    }

    // GP1-RB: Trigger auto-shoot sequence
    // Starts the 3-ball auto-shoot sequence (only works when flywheel is ON)
    private void handleAutoShootTrigger() {
        if (gamepad1.right_bumper && !lastGP1_RightBumper) {
            if (currentState == RobotState.SHOOTING && !autoShootActive) {
                // Start auto-shoot sequence
                startAutoShoot();
            } else if (autoShootActive) {
                // Cancel auto-shoot if already running
                cancelAutoShoot();
            }
        }
        lastGP1_RightBumper = gamepad1.right_bumper;
    }

    private void activateFlywheel() {
        launcher.setTargetRPM(flywheelRPM);  // Set target RPM for PIDF control
        launcher.setSpinning(true);
        launcher.setHoodPosition(hoodPosition);
        // Ramp will auto-raise when RPM reaches target (within 150 RPM)
        flywheelOn = true;
    }

    private void deactivateFlywheel() {
        launcher.setSpinning(false);
        intakeTransfer.transferDown();  // Automatically lower ramp when flywheel turns off
        rampUp = false;  // Reset ramp state
        rampAutoControlEnabled = false;  // Disable auto-control when flywheel turns off
        flywheelOn = false;
    }

    // Automatic ramp control - raises ramp when flywheel reaches target RPM (within 150 RPM)
    private void handleAutomaticRampControl() {
        if (!rampAutoControlEnabled) {
            return;  // Auto-control disabled, do nothing
        }

        // Use the public method for autonomous compatibility
        updateAutomaticRampControl();
    }

    // ========================================
    // AUTO-SHOOT SEQUENCE (3-BALL)
    // ========================================

    /**
     * Start the auto-shoot sequence
     * Waits for flywheel to reach target RPM, then fires 3 balls automatically
     * Waits for RPM recovery between each shot for consistency
     */
    private void startAutoShoot() {
        autoShootActive = true;
        waitingForFlywheelSpinup = true;
        autoShootPhase = 0;
        autoShootTimer.reset();
        intakeTransfer.stopIntake();  // Stop any ongoing intake

        // Ensure automatic ramp control is enabled
        if (!rampAutoControlEnabled) {
            rampAutoControlEnabled = true;
        }
    }

    /**
     * Cancel the auto-shoot sequence
     */
    private void cancelAutoShoot() {
        autoShootActive = false;
        waitingForFlywheelSpinup = false;
        autoShootPhase = 0;
        intakeTransfer.stopIntake();
    }

    /**
     * Handle the auto-shoot sequence timing
     * Sequence: Wait for RPM → Ball 1 (0.5s) → Wait for RPM → Ball 2 (0.5s) → Wait for RPM → Ball 3 (1.0s) → Done
     */
    private void handleAutoShootSequence() {
        if (!autoShootActive) {
            return;
        }

        double currentRPM = launcher.getCurrentRPM();
        double targetRPM = launcher.getTargetRPM();
        double error = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = error <= RPM_TOLERANCE;

        // Phase 0: Wait for initial flywheel spinup
        if (waitingForFlywheelSpinup) {
            if (rpmReady) {
                // Flywheel is ready! Start shooting sequence
                waitingForFlywheelSpinup = false;
                autoShootPhase = 1;
                autoShootTimer.reset();
            }
            return;  // Keep waiting
        }

        // Execute shooting sequence with RPM-based recovery
        double elapsed = autoShootTimer.milliseconds();

        switch (autoShootPhase) {
            case 1:  // Ball 1: Shoot for 0.5s
                if (elapsed < SHOT_DURATION_MS) {
                    intakeTransfer.startIntake(1.0);
                } else {
                    intakeTransfer.stopIntake();
                    autoShootPhase = 2;  // Move to recovery wait
                    autoShootTimer.reset();
                }
                break;

            case 2:  // Wait for RPM recovery after ball 1
                if (rpmReady) {
                    autoShootPhase = 3;  // RPM recovered, shoot ball 2
                    autoShootTimer.reset();
                }
                // Keep waiting for RPM to recover
                break;

            case 3:  // Ball 2: Shoot for 0.5s
                if (elapsed < SHOT_DURATION_MS) {
                    intakeTransfer.startIntake(1.0);
                } else {
                    intakeTransfer.stopIntake();
                    autoShootPhase = 4;  // Move to recovery wait
                    autoShootTimer.reset();
                }
                break;

            case 4:  // Wait for RPM recovery after ball 2
                if (rpmReady) {
                    autoShootPhase = 5;  // RPM recovered, shoot ball 3 (final)
                    autoShootTimer.reset();
                }
                // Keep waiting for RPM to recover
                break;

            case 5:  // Ball 3 (FINAL): Shoot for 1.0s
                if (elapsed < FINAL_SHOT_DURATION_MS) {
                    intakeTransfer.startIntake(1.0);
                } else {
                    // Sequence complete!
                    intakeTransfer.stopIntake();
                    autoShootActive = false;
                    autoShootPhase = 0;
                }
                break;
        }
    }

    // ========================================
    // GAMEPAD 2: MANUAL OVERRIDES
    // ========================================

    // GP2 Manual Overrides:
    // - DPAD UP/DOWN: Adjust RPM (±100 RPM per press)
    // - LEFT/RIGHT BUMPER: Adjust hood angle (±0.03 per press)
    // - START: Emergency stop
    private void handleManualOverrides() {
        // DPAD UP: Increase RPM by 100
        if (gamepad2.dpad_up && !lastGP2_DpadUp) {
            flywheelRPM += 100;
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
            }
            selectedPreset = String.format("MANUAL (%.0f RPM)", flywheelRPM);
        }
        lastGP2_DpadUp = gamepad2.dpad_up;

        // DPAD DOWN: Decrease RPM by 100
        if (gamepad2.dpad_down && !lastGP2_DpadDown) {
            flywheelRPM = Math.max(1000, flywheelRPM - 100);
            if (flywheelOn) {
                launcher.setTargetRPM(flywheelRPM);
            }
            selectedPreset = String.format("MANUAL (%.0f RPM)", flywheelRPM);
        }
        lastGP2_DpadDown = gamepad2.dpad_down;

        // B: Lower hood (decrease angle, flatter trajectory)
        if (gamepad2.b && !lastGP2_B) {
            hoodPosition = Math.max(0.0, hoodPosition - 0.03);
            launcher.setHoodPosition(hoodPosition);
        }
        lastGP2_B = gamepad2.b;

        // Y: Raise hood (increase angle, steeper trajectory)
        if (gamepad2.y && !lastGP2_Y) {
            hoodPosition = Math.min(1.0, hoodPosition + 0.03);
            launcher.setHoodPosition(hoodPosition);
        }
        lastGP2_Y = gamepad2.y;
    }

    // ========================================
    // TELEMETRY & SHUTDOWN
    // ========================================

    private void updateTelemetry() {
        // === HEADER ===
        telemetry.addLine("=== " + alliance.name() + " TELEOP ===");

        // === ROBOT STATE ===
        telemetry.addData("State", "%s (Speed: 100%%)", currentState);

        // === EJECT-SHOOT SEQUENCE ===
        if (ejectShootActive) {
            long elapsed = System.currentTimeMillis() - ejectShootStartTime;
            if (elapsed < EJECT_DURATION_MS) {
                telemetry.addLine(">>> EJECTING BALL <<<");
            } else {
                telemetry.addLine(">>> SHOOTING BALL <<<");
            }
        }

        // === AUTO-SHOOT SEQUENCE (3-BALL) ===
        if (autoShootActive) {
            if (waitingForFlywheelSpinup) {
                double currentRPM = launcher.getCurrentRPM();
                double targetRPM = launcher.getTargetRPM();
                double error = Math.abs(targetRPM - currentRPM);
                telemetry.addLine(String.format(">>> WAITING FOR FLYWHEEL: %.0f RPM (%.0f to go) <<<",
                        currentRPM, error));
            } else {
                // Phase 1=shoot1, 2=wait, 3=shoot2, 4=wait, 5=shoot3
                String status;
                switch (autoShootPhase) {
                    case 1: status = "BALL 1/3 - SHOOTING"; break;
                    case 2: status = "RECOVERING RPM..."; break;
                    case 3: status = "BALL 2/3 - SHOOTING"; break;
                    case 4: status = "RECOVERING RPM..."; break;
                    case 5: status = "BALL 3/3 - FINAL SHOT"; break;
                    default: status = "AUTO-SHOOT"; break;
                }
                telemetry.addLine(String.format(">>> AUTO-SHOOT: %s <<<", status));
            }
        }

        // === EMERGENCY STOP WARNING ===
        if (emergencyStopped) {
            telemetry.addLine("*** E-STOP ACTIVE - Release GP2-START ***");
        }

        // === SHOOTING STATUS ===
        if (flywheelOn) {
            double currentRPM = launcher.getCurrentRPM();
            double error = Math.abs(flywheelRPM - currentRPM);
            boolean ready = error < 50;  // Ready if within 50 RPM
            boolean boosting = launcher.isInBoostPhase();

            String status;
            if (boosting) {
                status = "BOOST SPINUP...";
            } else if (ready) {
                status = ">>> READY TO SHOOT <<<";
            } else {
                status = "SPINNING UP...";
            }
            telemetry.addData("Flywheel", status);
            telemetry.addData("Target/Actual", "%.0f / %.0f RPM", flywheelRPM, currentRPM);

            // Gamepad rumble when ready
            if (ready && error < 30) {
                gamepad1.rumble(100);  // Quick pulse when locked on speed
            }
        } else {
            telemetry.addData("Flywheel", "OFF (GP1-Y to turn on)");
        }

        // === RAMP STATUS ===
        String rampStatus;
        if (rampAutoControlEnabled) {
            if (rampUp) {
                rampStatus = "UP (ready to shoot)";
            } else {
                if (flywheelOn) {
                    double currentRPM = launcher.getCurrentRPM();
                    double error = Math.abs(flywheelRPM - currentRPM);
                    if (!inLaunchZone) {
                        rampStatus = "DOWN (MOVE TO TRIANGLE ZONE!)";
                    } else {
                        rampStatus = String.format("DOWN (spinning up... %.0f RPM to go)", error);
                    }
                } else {
                    rampStatus = "DOWN (press Y to start)";
                }
            }
        } else {
            rampStatus = rampUp ? "UP" : "DOWN (press Y to start)";
        }
        telemetry.addData("Ramp", rampStatus);

        telemetry.addData("Hood", "%.2f", hoodPosition);
        telemetry.addData("Preset", selectedPreset);

        // === DISTANCE INFO ===
        telemetry.addData("Distance Detect", distanceDetectionEnabled ? "ON (auto-adjusting)" : "OFF (manual)");
        if (distanceDetectionEnabled && currentDistance > 0) {
            telemetry.addData("Current Dist", "%.2fft (auto-applied)", currentDistance);
        }
        if (lockedDistance > 0 && distanceDetectionEnabled) {
            telemetry.addData("Applied Dist", "%.2fft", lockedDistance);
        }

        // === POSITION & LAUNCH ZONE ===
        if (pinpoint != null) {
            telemetry.addData("Position", "X: %.1f\" Y: %.1f\"", robotX, robotY);

            // Detailed zone check
            boolean check1 = robotX >= -30;
            boolean check2 = robotY >= -30;
            boolean check3 = robotY <= 174;
            boolean check4 = robotY >= robotX - 30;

            String zoneStatus;
            if (inLaunchZone) {
                zoneStatus = ">>> IN ZONE <<<";
            } else {
                // Show which condition is failing
                if (!check1) zoneStatus = "X too low (need X >= -30)";
                else if (!check2) zoneStatus = "Y too low (need Y >= -30)";
                else if (!check3) zoneStatus = "Y too high (need Y <= 174)";
                else if (!check4) zoneStatus = String.format("Below diagonal (Y < X-30)");
                else zoneStatus = "OUTSIDE - move to triangle!";
            }

            telemetry.addData("Launch Zone", zoneStatus);

            // Extra debug info when enabled
            if (SHOW_ZONE_DEBUG) {
                telemetry.addLine();
                telemetry.addLine("--- ZONE DEBUG ---");
                telemetry.addData("X >= -30", check1 ? "PASS" : "FAIL");
                telemetry.addData("Y >= -30", check2 ? "PASS" : "FAIL");
                telemetry.addData("Y <= 174", check3 ? "PASS" : "FAIL");
                telemetry.addData("Y >= X-30", check4 ? "PASS" : String.format("FAIL (%.1f < %.1f)", robotY, robotX - 30));
                telemetry.addData("Result", inLaunchZone ? "IN ZONE" : "OUT");
            }
        } else {
            telemetry.addData("Pinpoint", "NOT FOUND (ramp unrestricted)");
        }

        // === TURRET STATUS ===
        String turretMode = turretTracking ? (turret.isLocked() ? "LOCKED" : "TRACKING") : "MANUAL";
        telemetry.addData("Turret", "%s (%.2f)", turretMode, turret.getServoPosition());
        telemetry.addData("Turret Mode", turret.isFieldGoalAutoAlignEnabled() ? "FIELD GOAL" : "APRILTAG");
        telemetry.addData("Turret Debug", turret.getDebugState());

        // === DEBUG INFO (Optional) ===
        if (SHOW_DEBUG_INFO) {
            telemetry.addLine();
            telemetry.addData("Limelight", getLimelightDebugInfo());
        }

        // === CONTROLS (Compact) ===
        telemetry.addLine();
        telemetry.addLine("GP1: Y-Flywheel+Ramp RB-AutoShoot LB-EjectShoot | RT-In LT-Out | DPAD:U-Auto D-Default R-Far");
        telemetry.addLine("GP2: RB-GoalAlign X-CtrTur | LS-Turret DPAD:L-Tur U/D-RPM | B/Y-Hood | START-ESTOP");

        telemetry.update();
    }

    // ========================================
    // PUBLIC METHODS FOR AUTONOMOUS USE
    // ========================================

    /**
     * Automatic ramp control based on flywheel RPM AND robot position
     * Call this in a loop during autonomous to automatically manage the ramp
     *
     * Launch zone: Triangle with vertices (0,0), (0,144), (144,144) plus 30" tolerance
     *
     * Uses hysteresis to prevent oscillation:
     * - Raises ramp when flywheel is within 150 RPM of target AND robot is in launch zone
     * - Lowers ramp only when flywheel drops beyond 300 RPM of target OR robot leaves launch zone
     *
     * @return true if ramp is up and ready to shoot, false otherwise
     */
    public boolean updateAutomaticRampControl() {
        // Calculate flywheel error
        double error = 0;
        if (flywheelOn) {
            double currentRPM = launcher.getCurrentRPM();
            error = Math.abs(flywheelRPM - currentRPM);
        }

        // Hysteresis logic to prevent oscillation
        if (rampUp) {
            // Ramp is currently UP - only lower if RPM drops significantly (beyond hysteresis threshold)
            // OR if robot leaves the launch zone
            if (!flywheelOn || error > RPM_HYSTERESIS || !inLaunchZone) {
                intakeTransfer.transferDown();
                rampUp = false;
            }
        } else {
            // Ramp is currently DOWN - raise when RPM is within tolerance AND robot is in launch zone
            if (flywheelOn && error <= RPM_TOLERANCE && inLaunchZone) {
                intakeTransfer.transferUp();
                rampUp = true;
            }
        }

        // Return true only if ramp is up AND flywheel is within tolerance AND in launch zone
        return rampUp && flywheelOn && (error <= RPM_TOLERANCE) && inLaunchZone;
    }

    /**
     * Start the automatic 3-ball shooting sequence
     * Call this once to start, then call updateAutoShootSequence() in a loop
     *
     * Sequence waits for RPM to be within MOE (150 RPM) before each shot:
     * - Wait for initial spinup (within 150 RPM)
     * - Ball 1: Shoot for 0.5s
     * - Wait for RPM recovery (within 150 RPM)
     * - Ball 2: Shoot for 0.5s
     * - Wait for RPM recovery (within 150 RPM)
     * - Ball 3: Shoot for 1.0s
     */
    public void startAutoShootSequence() {
        autoShootActive = true;
        waitingForFlywheelSpinup = true;
        autoShootPhase = 0;
        autoShootTimer.reset();
        intakeTransfer.stopIntake();

        // Enable automatic ramp control
        rampAutoControlEnabled = true;
    }

    /**
     * Update the auto-shoot sequence
     * Call this in a loop after calling startAutoShootSequence()
     *
     * Uses RPM-based timing with MOE of 150 RPM between shots
     *
     * @return true if sequence is complete, false if still running
     */
    public boolean updateAutoShootSequence() {
        if (!autoShootActive) {
            return true;  // Already complete
        }

        handleAutoShootSequence();  // Run the sequence logic

        return !autoShootActive;  // Returns true when complete
    }

    /**
     * Check if auto-shoot sequence is currently active
     *
     * @return true if auto-shoot is running, false otherwise
     */
    public boolean isAutoShootActive() {
        return autoShootActive;
    }

    /**
     * Cancel the auto-shoot sequence
     * Stops intake and resets sequence state
     */
    public void cancelAutoShootSequence() {
        cancelAutoShoot();
    }

    /**
     * Enable or disable automatic ramp control
     * When enabled, ramp automatically raises/lowers based on RPM (within 150 RPM MOE)
     *
     * @param enabled true to enable automatic control, false to disable
     */
    public void setRampAutoControlEnabled(boolean enabled) {
        rampAutoControlEnabled = enabled;

        // If disabling, lower the ramp immediately
        if (!enabled && rampUp) {
            intakeTransfer.transferDown();
            rampUp = false;
        }
    }

    /**
     * Check if automatic ramp control is enabled
     *
     * @return true if auto-control is enabled, false otherwise
     */
    public boolean isRampAutoControlEnabled() {
        return rampAutoControlEnabled;
    }

    /**
     * Get the current RPM margin of error tolerance
     * This is the acceptable difference between target and actual RPM
     *
     * @return RPM tolerance (MOE = 150)
     */
    public double getRPMTolerance() {
        return RPM_TOLERANCE;
    }

    // ========================================
    // SHUTDOWN
    // ========================================

    private void shutdown() {
        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
        turret.stop();
    }
}
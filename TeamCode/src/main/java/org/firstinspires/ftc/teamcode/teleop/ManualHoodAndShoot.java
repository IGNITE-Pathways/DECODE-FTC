package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hood and Shoot Control", group="Individual Test")
public class ManualHoodAndShoot extends OpMode {

    private Launcher launcherComponent;
    private Limelight3A limelight;
    
    // Odometry tracking via IMU with odometry pods
    private GoBildaPinpointDriver imu;
    
    // Kicker servo
    private Servo kickerServo;
    private static final double KICKER_RESET_POSITION = 0.89;
    private static final double KICKER_FLICK_POSITION = 0.98;
    
    // Kicker sequence state
    private boolean kickerSequenceActive = false;
    private long kickerSequenceStartTime = 0;
    private int kickerSequenceStage = 0; // 0 = waiting, 1 = delay before flick, 2 = flick, 3 = delay before reset, 4 = reset
    
    // Rate limiting for triggers
    private long lastTriggerUpdateTime = 0;
    private static final long TRIGGER_UPDATE_INTERVAL_MS = 50; // Update every 50ms for smoother control
    
    private double odoX = 0.0; // Starting at 0, 0
    private double odoY = 0.0;
    
    // Drift prevention
    private double lastOdoX = 0.0;
    private double lastOdoY = 0.0;
    private static final double DRIFT_THRESHOLD = 0.05; // Ignore movements smaller than 0.05 inches
    private int stationaryCount = 0;
    private static final int STATIONARY_RESET_THRESHOLD = 50; // Reset if stationary for 50 loops (~1 second)
    
    // Limelight constants (matching AutoHoodAndShoot)
    private static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032; // 8 inches before now i made it 30
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;   // Limelight 3A vertical FOV
    private static final int IMAGE_WIDTH_PIXELS = 1280;
    private static final int IMAGE_HEIGHT_PIXELS = 720;
    
    private double flywheelPower = 0.1; // starting power
    private boolean spinning = false;   // flywheel state
    private final double HOOD_INCREMENT = 0.1;
    
    // Edge detection for button debouncing
    private boolean lastTriangle = false;
    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean lastCircle = false;

    @Override
    public void init() {
        launcherComponent = new Launcher();
        launcherComponent.initialize(hardwareMap, telemetry);
        launcherComponent.setPower(flywheelPower);
        
        // Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.start();
        }
        
        // Initialize odometry pods via IMU
        try {
            imu = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
            if (imu != null) {
                // Configure odometry pods using values from Constants
                // Pod offsets: forwardPodY = 3.75, strafePodX = -7.08661
                imu.setOffsets(-7.08661, 3.75, DistanceUnit.INCH);
                
                // Set encoder resolution to goBILDA_4_BAR_POD
                imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                
                // Set encoder directions (both FORWARD)
                imu.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,  // forward encoder
                    GoBildaPinpointDriver.EncoderDirection.FORWARD   // strafe encoder
                );
                
                // Calibrate IMU and reset position (important for preventing drift)
                imu.resetPosAndIMU();
                
                // Set initial position to (0, 0, 0)
                imu.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                telemetry.addData("Odometry", "Initialized - Starting at (0, 0)");
                telemetry.addData("Odometry", "IMU calibrated and reset");
            } else {
                telemetry.addData("Odometry", "ERROR: IMU not found!");
            }
        } catch (Exception e) {
            telemetry.addData("Odometry", "ERROR: " + e.getMessage());
            imu = null;
        }
        
        // Initialize kicker servo
        try {
            kickerServo = hardwareMap.get(Servo.class, "kicker");
            if (kickerServo != null) {
                kickerServo.setPosition(KICKER_RESET_POSITION);
                telemetry.addData("Kicker", "Initialized");
            } else {
                telemetry.addData("Kicker", "ERROR: Kicker servo not found!");
            }
        } catch (Exception e) {
            telemetry.addData("Kicker", "ERROR: " + e.getMessage());
            kickerServo = null;
        }
        
        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.addData("Status", "Initialized. Servo position: %.2f", launcherComponent.getHoodPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry from IMU with odometry pods (with drift filtering)
        if (imu != null) {
            imu.update();
            Pose2D pose = imu.getPosition();
            double rawX = pose.getX(DistanceUnit.INCH);
            double rawY = pose.getY(DistanceUnit.INCH);
            
            // Calculate movement delta
            double deltaX = rawX - lastOdoX;
            double deltaY = rawY - lastOdoY;
            double movement = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // Only update if movement is significant (above drift threshold)
            if (movement >= DRIFT_THRESHOLD) {
                odoX = rawX;
                odoY = rawY;
                lastOdoX = rawX;
                lastOdoY = rawY;
                stationaryCount = 0;
            } else {
                // Robot appears stationary - increment counter
                stationaryCount++;
                
                // If stationary for too long, reset position to prevent accumulated drift
                if (stationaryCount >= STATIONARY_RESET_THRESHOLD) {
                    // Reset IMU position to current tracked position to prevent drift accumulation
                    imu.setPosition(new Pose2D(DistanceUnit.INCH, odoX, odoY, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES)));
                    lastOdoX = odoX;
                    lastOdoY = odoY;
                    stationaryCount = 0;
                }
            }
        }
        
        // Handle kicker sequence state machine
        if (kickerSequenceActive && kickerServo != null) {
            long currentTime = System.currentTimeMillis();
            long elapsedTime = currentTime - kickerSequenceStartTime;
            
            switch (kickerSequenceStage) {
                case 1: // Delay before flick (1000ms)
                    if (elapsedTime >= 1000) {
                        kickerServo.setPosition(KICKER_FLICK_POSITION);
                        kickerSequenceStage = 2;
                        kickerSequenceStartTime = currentTime;
                    }
                    break;
                case 2: // Flick position (1200ms)
                    if (elapsedTime >= 1200) {
                        kickerServo.setPosition(KICKER_RESET_POSITION);
                        kickerSequenceStage = 3;
                        kickerSequenceStartTime = currentTime;
                    }
                    break;
                case 3: // Delay after reset (1000ms)
                    if (elapsedTime >= 1000) {
                        kickerSequenceActive = false;
                        kickerSequenceStage = 0;
                    }
                    break;
            }
        }
        
        // Calculate limelight distance (using TA method from AutoHoodAndShoot)
        double limelightDistance = -1.0;
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
        
        // Trigger kicker sequence with X (Cross) - edge detection (takes priority)
        boolean kickerTriggered = false;
        if (gamepad1.cross && !lastCross) {
            if (!kickerSequenceActive && kickerServo != null) {
                kickerSequenceActive = true;
                kickerSequenceStage = 1;
                kickerSequenceStartTime = System.currentTimeMillis();
                kickerTriggered = true;
            }
        }

        // Increase flywheel RPM with Left Trigger (rate-limited continuous)
        long currentTime = System.currentTimeMillis();
        if (gamepad1.left_trigger > 0.2) {
            if (currentTime - lastTriggerUpdateTime >= TRIGGER_UPDATE_INTERVAL_MS) {
                flywheelPower += 0.02; // Increment at controlled rate
                flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));
                launcherComponent.setPower(flywheelPower);
                lastTriggerUpdateTime = currentTime;
            }
        }
        
        // Decrease flywheel RPM with Left Bumper (rate-limited continuous)
        if (gamepad1.left_bumper) {
            if (currentTime - lastTriggerUpdateTime >= TRIGGER_UPDATE_INTERVAL_MS) {
                flywheelPower -= 0.02; // Decrement at controlled rate
                flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));
                launcherComponent.setPower(flywheelPower);
                lastTriggerUpdateTime = currentTime;
            }
        }
        
        // Toggle flywheel on/off with Circle - edge detection
        if (gamepad1.circle && !lastCircle) {
            spinning = !spinning;
            launcherComponent.setSpinning(spinning);
        }

        // Update launcher (flywheel)
        launcherComponent.update();

        // Increase hood with Right Trigger (rate-limited continuous)
        if (gamepad1.right_trigger > 0.2) {
            if (currentTime - lastTriggerUpdateTime >= TRIGGER_UPDATE_INTERVAL_MS) {
                launcherComponent.incrementHood();
                lastTriggerUpdateTime = currentTime;
            }
        }
        
        // Decrease hood with Right Bumper (rate-limited continuous)
        if (gamepad1.right_bumper) {
            if (currentTime - lastTriggerUpdateTime >= TRIGGER_UPDATE_INTERVAL_MS) {
                launcherComponent.decrementHood();
                lastTriggerUpdateTime = currentTime;
            }
        }

        // Update edge detection states
        lastTriangle = gamepad1.triangle;
        lastCross = gamepad1.cross;
        lastSquare = gamepad1.square;
        lastCircle = gamepad1.circle;

        // Telemetry - Display all data for prediction model
        telemetry.addLine("=== PREDICTION MODEL DATA ===");
        telemetry.addData("Odo X (in)", "%.4f", odoX);
        telemetry.addData("Odo Y (in)", "%.4f", odoY);
        telemetry.addData("Limelight Distance (ft)", limelightDistance > 0 ? String.format("%.4f", limelightDistance) : "N/A");
        telemetry.addData("Hood Pos", "%.4f", launcherComponent.getHoodPosition());
        telemetry.addData("Flywheel Speed", "%.4f", flywheelPower);
        telemetry.addLine("");
        telemetry.addLine("=== CONTROL STATUS ===");
        telemetry.addData("Status", spinning ? "Shooting" : "Stopped");
        telemetry.addData("Kicker", kickerSequenceActive ? "Active" : "Ready");
        telemetry.update();
    }
    
    @Override
    public void stop() {
        // Stop limelight
        if (limelight != null) {
            limelight.stop();
        }
    }
}

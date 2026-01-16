package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.constants.Shooter;
//import org.firstinspires.ftc.teamcode.ProgrammingBoard.ProgrammingBoardOTHER;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "FlyWheel Distance Shooter", group = "Main")
public class FlyWheelDistanceShooter extends OpMode {

//    private ProgrammingBoardOTHER board = new ProgrammingBoardOTHER();
    private Limelight3A limelight;

    private double flywheelPower = 0.1;  // starting power
    private boolean spinning = false;    // flywheel state

    // Rolling average for distance calculation
    private static final int MAX_SAMPLES = 20; // Sample over ~2 seconds at ~10Hz
    private static final long SAMPLE_INTERVAL_MS = 100; // Sample every 100ms
    private Queue<Double> distanceSamples = new LinkedList<>();
    private ElapsedTime sampleTimer = new ElapsedTime();

    // Use shared Shooter constants
    private static final double CAMERA_HEIGHT_METERS = Shooter.CAMERA_HEIGHT_METERS;
    private static final double TARGET_HEIGHT_METERS = Shooter.TARGET_HEIGHT_METERS;
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = Shooter.CAMERA_MOUNT_ANGLE_DEGREES;

    @Override
    public void init() {
//        board.initializeComponents(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        telemetry.addData("Status", "Initialized. Flywheel power: %.2f", flywheelPower);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Take average distance over ~2 seconds
        double sumDistanceFeet = 0.0;
        int count = 0;
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 2000) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double verticalOffsetDeg = result.getTy();
                double totalAngleDeg = CAMERA_MOUNT_ANGLE_DEGREES + verticalOffsetDeg;
                double totalAngleRad = Math.toRadians(totalAngleDeg);

                double distanceMeters = (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(totalAngleRad);
                double distanceFeet = distanceMeters * 3.28084;

                sumDistanceFeet += distanceFeet;
                count++;
            }
        }

        double avgDistanceFeet = count > 0 ? sumDistanceFeet / count : 0.0;

        // Power scales +0.1 per foot, capped at 1.0
        double distanceBasedPower = Math.min(1.0, 0.1 * avgDistanceFeet);

        // Flywheel control
        if (gamepad1.square) {
            spinning = true;
            flywheelPower = distanceBasedPower; // set power based on distance
        }
        if (gamepad1.circle) {
            spinning = false;
        }

        // Manual fine adjustments
        if (gamepad1.triangle) flywheelPower = Math.min(1.0, flywheelPower + 0.05);
        if (gamepad1.cross) flywheelPower = Math.max(0.0, flywheelPower - 0.05);

        // Apply power
//        board.flyWheelMotor.setPower(spinning ? flywheelPower : 0);

        telemetry.addData("Average Distance (ft)", "%.2f", avgDistanceFeet);
        telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
        telemetry.addData("Spinning", spinning ? "Yes" : "No");
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.constants.Shooter;

@TeleOp(name = "Hood Camera Control", group = "Main")
public class HoodCameraControl extends LinearOpMode {

    private Limelight3A limelight;
    private Servo hoodServo;
    private static final double CAMERA_HEIGHT_METERS = Shooter.CAMERA_HEIGHT_METERS;
    private static final double TARGET_HEIGHT_METERS = Shooter.TARGET_HEIGHT_METERS;
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = Shooter.CAMERA_MOUNT_ANGLE_DEGREES;

    // Servo control
    private double servoPosition = 0.0;
    private static final double SERVO_STEP_PER_FOOT = 0.1;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hoodServo = hardwareMap.get(Servo.class, "hoodservo");

        if (hoodServo != null) {
            hoodServo.setPosition(servoPosition);
        }
        if (limelight != null) {
            limelight.start();
        }

        telemetry.addLine("Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double distanceFeet = -1;

            if (result != null && result.isValid()) {
                double verticalOffsetDeg = result.getTy();
                double totalAngleDeg = CAMERA_MOUNT_ANGLE_DEGREES + verticalOffsetDeg;
                double totalAngleRad = Math.toRadians(totalAngleDeg);

                double distanceMeters = (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(totalAngleRad);
                distanceFeet = distanceMeters * 3.28084;

                telemetry.addData("Distance to Target (ft)", "%.2f", distanceFeet);
                telemetry.addData("Vertical Offset (deg)", "%.2f", verticalOffsetDeg);
                telemetry.addData("Total Angle (deg)", "%.2f", totalAngleDeg);
            } else {
                telemetry.addLine("No valid target detected");
            }

            // Adjust hood angle when X (cross) is pressed
            if (gamepad1.cross && distanceFeet > 0) {
                servoPosition = SERVO_STEP_PER_FOOT * (distanceFeet-5);
                servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

                hoodServo.setPosition(servoPosition);
                telemetry.addData("Servo Adjusted", "Set to %.2f based on %.2f ft", servoPosition, distanceFeet);
                sleep(200);
            }

            telemetry.addData("Current Servo Pos", "%.2f", servoPosition);
            telemetry.update();
        }

        limelight.stop();
    }
}

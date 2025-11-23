package OpModes.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import ProgrammingBoard.ProgrammingBoardOTHER;

@TeleOp(name = "Auto Hood and Shoot", group = "Main")
public class AutoHoodAndShoot extends LinearOpMode {

    private Limelight3A limelight;
    private Servo hoodServo;
    private ProgrammingBoardOTHER board = new ProgrammingBoardOTHER();

    // --- State ---
    private double servoPosition = 0.0;
    private double flywheelPower = 0.0;
    private boolean hoodActive = true;
    private boolean shooterActive = true;
    private boolean bothActive = true;

    // --- Constants ---
    private static final double SERVO_STEP_PER_FOOT = 0.09;
    private static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032;; // 8 inches before now i made it 30
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;   // Limelight 3A vertical FOV
    private static final int IMAGE_WIDTH_PIXELS = 1280;
    private static final int IMAGE_HEIGHT_PIXELS = 720;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hoodServo = hardwareMap.get(Servo.class, "hoodservo");
        board.initializeComponents(hardwareMap);
        limelight.start();

        telemetry.addLine("Auto Hood + Shoot Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double distanceFeet = -1;

            // --- Distance estimation via TA (AprilTag area) ---
            if (result != null && result.isValid()) {
                double taPercent = result.getTa();
                if (taPercent > 0.0) {
                    double pixelArea = (taPercent / 100.0) * (IMAGE_WIDTH_PIXELS * IMAGE_HEIGHT_PIXELS);
                    double tagPixelHeight = Math.sqrt(pixelArea);
                    double focalPx = (IMAGE_HEIGHT_PIXELS / 2.0)
                            / Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV_DEGREES / 2.0));

                    double distanceMeters = (APRILTAG_REAL_HEIGHT_METERS * focalPx) / tagPixelHeight;
                    distanceFeet = distanceMeters * 3.28084;

                    telemetry.addData("Distance (ft) [TA]", "%.2f", distanceFeet);
                } else {
                    telemetry.addLine("No valid target area data");
                }
            } else {
                telemetry.addLine("No valid target detected");
            }

            // --- X / Square button → both hood + shooter active ---
            if (gamepad1.square && distanceFeet > 0) {
                bothActive = true;
                hoodActive = true;
                shooterActive = true;
            }

            // --- Left Trigger → hood active ---
            if (gamepad1.left_trigger > 0.2 && distanceFeet > 0) {
                hoodActive = true;
            }

            // --- Right Trigger → shooter active ---
            if (gamepad1.right_trigger > 0.2 && distanceFeet > 0) {
                shooterActive = true;
            }

            // --- Update hood if active ---
            if (hoodActive && distanceFeet > 0) {
                servoPosition = SERVO_STEP_PER_FOOT * (distanceFeet-1);
                servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
                hoodServo.setPosition(servoPosition);
            }

            // --- Update shooter if active ---
            if (shooterActive && distanceFeet > 0) {
                    flywheelPower = Math.min(1.0, ((0.085 * distanceFeet)));

                board.flyWheelMotor.setPower(flywheelPower);
                board.flyWheelMotor2.setPower(flywheelPower);
            }

            // --- Keep servo powered always (holding position) ---
            hoodServo.setPosition(servoPosition);

            telemetry.addData("Servo Pos", "%.2f", servoPosition);
            telemetry.addData("Shooter Power", "%.2f", flywheelPower);
            telemetry.addData("Hood Active", hoodActive);
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.addData("Both Active", bothActive);
            telemetry.update();
        }

        // Stop shooter at end of OpMode
        board.flyWheelMotor.setPower(0);
        board.flyWheelMotor2.setPower(0);
        limelight.stop();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Main.Components.LimelightLocalization;

/**
 * Test OpMode for LimelightLocalization with GoBilda Pinpoint
 *
 * Shows:
 * - Pinpoint X, Y, Heading (odometry)
 * - Distance to visible AprilTags
 * - Target area for calibration
 *
 * CONTROLS:
 * - A: Reset heading to 0
 * - B: Reset position (X, Y, Heading all to 0)
 * - Y: Recalibrate IMU (robot must be stationary)
 *
 * APRILTAG IDS:
 * - 20: Blue Alliance
 * - 24: Red Alliance
 * - 21, 22, 23: Obelisk
 */
@TeleOp(name = "Test: Limelight Localization", group = "Test")
public class LimelightLocalizationTest extends LinearOpMode {

    private LimelightLocalization localization;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {
        localization = new LimelightLocalization();
        localization.initialize(hardwareMap, telemetry);
        telemetry.addLine("=== Limelight + Pinpoint Test ===");
        telemetry.addLine("");
        telemetry.addLine("Shows:");
        telemetry.addLine("  - Pinpoint X, Y, Heading");
        telemetry.addLine("  - Distance to tags");
        telemetry.addLine("  - Target area (for calibration)");
        telemetry.addLine("");
        telemetry.addLine("A = Reset heading");
        telemetry.addLine("B = Reset position (all to 0)");
        telemetry.addLine("Y = Recalibrate IMU (stay still!)");
        telemetry.addLine("");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Reset heading on A press
            if (gamepad1.a && !prevA) {
                localization.resetHeading();
            }
            prevA = gamepad1.a;

            // Reset position on B press
            if (gamepad1.b && !prevB) {
                localization.resetPosition();
            }
            prevB = gamepad1.b;

            // Recalibrate IMU on Y press
            if (gamepad1.y && !prevY) {
                localization.recalibrateIMU();
            }
            prevY = gamepad1.y;

            // Update localization
            boolean hasValidData = localization.update();

            // Display results
            telemetry.addLine("======== LOCALIZATION ========");
            telemetry.addLine("");

            // Connection status
            telemetry.addData("Limelight", localization.isConnected() ? "CONNECTED" : "DISCONNECTED");
            telemetry.addData("Pinpoint", localization.isPinpointConnected() ? "CONNECTED" : "DISCONNECTED");
            if (localization.isPinpointConnected()) {
                telemetry.addData("Status", localization.getPinpointStatus());
            }
            telemetry.addLine("");

            // Pinpoint odometry data
            telemetry.addLine("--- PINPOINT ODOMETRY ---");
            telemetry.addData("X", "%.2f in", localization.getX());
            telemetry.addData("Y", "%.2f in", localization.getY());
            telemetry.addData("Heading", "%.1f°", localization.getHeading());
            telemetry.addLine("");

            // Tag info
            telemetry.addLine("--- APRILTAGS ---");
            telemetry.addData("Tags Visible", localization.getTagCount());

            if (localization.isTagVisible()) {
                telemetry.addData("Primary Tag", localization.getPrimaryTagId());
                double distIn = localization.getDistance();
                telemetry.addData("Distance", "%.1f in (%.2f ft)", distIn, distIn / 12.0);
                telemetry.addData("Angle (tx)", "%.1f°", localization.getTx());
                telemetry.addData("3D Pose", "X:%.1f Y:%.1f Z:%.1f",
                        localization.getTagX(), localization.getTagY(), localization.getTagZ());
                telemetry.addData("Target Area", "%.3f%%", localization.getTargetArea());
            } else {
                telemetry.addLine("No tag visible - point at AprilTag");
            }
            telemetry.addLine("");

            // Check specific tags
            telemetry.addLine("--- TAG STATUS ---");
            telemetry.addData("Blue (20)", localization.isTagVisible(20) ?
                    String.format("%.1f in", localization.getDistanceToTag(20)) : "not visible");
            telemetry.addData("Red (24)", localization.isTagVisible(24) ?
                    String.format("%.1f in", localization.getDistanceToTag(24)) : "not visible");
            telemetry.addData("Obelisk 21", localization.isTagVisible(21) ? "VISIBLE" : "-");
            telemetry.addData("Obelisk 22", localization.isTagVisible(22) ? "VISIBLE" : "-");
            telemetry.addData("Obelisk 23", localization.isTagVisible(23) ? "VISIBLE" : "-");
            telemetry.addLine("");

            // Performance
            telemetry.addData("Latency", "%.0f ms", localization.getLatencyMs());
            telemetry.addLine("");
            telemetry.addLine("A=Reset heading | B=Reset position");
            telemetry.addLine("Y=Recalibrate IMU (stay still!)");

            telemetry.update();
            idle();
        }
    }
}
